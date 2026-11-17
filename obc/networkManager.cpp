#include "NetworkManager.h"
#include "esp_wifi.h"
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// Declaradas em telemetria.h/.cpp (forward para evitar include circular)
bool telemetriaIsImageSending();
bool opConsultarAeronave(double lat, double lon, const char* icao24, int photoIndex);

int nextIndexToRequest = 0;
uint32_t lastTotalIndex = 0xFFFFFFFF;

unsigned long lastProcessTime = 0;
const unsigned long PROCESS_DELAY = 60000;

uint8_t fb_buf[MAX_IMG_SIZE];

WiFiClient client;

volatile bool imageReady = false;

bool conectedOnce = false;
bool firstConnectionEver = true;

size_t currentImgSize = 0;
uint32_t currentTotalIndex = 0;
float currentMissionTime = 0.0;

//Dados do Wifi da Missão (AP da ESP32-CAM)
const char* ssid = "AMARAL_I";
const char* password = "uerjsats123";
const char* serverIP = "192.168.4.1";
const uint16_t port = 8888;

// ===========================================================================
// Missao secundaria: configuracao
// ===========================================================================
#define ADSB_UDP_PORT              4444     // broadcast da CAM {lat, lon, photo_index, photo_size}
#define M2_ENVIAR_SE_INDETERMINADO 1        // 1 = Orange nao respondeu -> manda a foto mesmo assim
#define M2_TIMEOUT_MS              40000UL  // tempo maximo preso numa deteccao
#define M2_TENTATIVAS_FOTO         3        // tentativas de download por TCP
#define M2_INTERVALO_FOTO_MS       1000UL   // espera entre tentativas de download
#define M2_FILA_DETECCOES          4        // deteccoes aguardando

static WiFiUDP udp;
static bool udpAberto = false;

// ===========================================================================
// Rede
// ===========================================================================
void setupNetwork()
{
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    Serial.println("conectando em...");

    unsigned long startAttemptTime = millis();

    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 15000)
    {
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    if (WiFi.status() == WL_CONNECTED)
        Serial.println("Conectado com Sucesso");
    else
        Serial.println("falha de conexao");
}

void resetRadio()
{
    Serial.println("resetando radio e buffers TCP");

    conectedOnce = false;

    client.stop();
    udp.stop();
    udpAberto = false;              // pollUdp() reabre quando o Wi-Fi voltar

    esp_wifi_stop();
    esp_wifi_deinit();

    vTaskDelay(pdMS_TO_TICKS(1500));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();

    WiFi.begin(ssid, password);

    Serial.println("Radio reiniciado");
}

void monitorConnection()
{
    // reconecta TCP
    if (WiFi.status() == WL_CONNECTED && !client.connected())
    {
        if (client.connect(serverIP, port))
        {
            client.setNoDelay(true);
            client.setTimeout(4000);

            // RESET DE MISSAO (MESMO FLUXO)
            if (firstConnectionEver)
            {
                client.println("R");
                client.flush();

                firstConnectionEver = false;

                Serial.println("primeira conexao — resetando servidor");

                // espera sem travar watchdog
                unsigned long startWait = millis();
                while (millis() - startWait < 5000)
                    vTaskDelay(pdMS_TO_TICKS(50));
            }

            conectedOnce = true;
        }
    }

    // queda de conexao
    if (conectedOnce && (WiFi.status() != WL_CONNECTED || !client.connected()))
    {
        resetRadio();
    }
}

// ===========================================================================
// Download de imagem por TCP
// ===========================================================================

// Descarta 'tamanho' bytes do socket (foto que nao sera usada)
static void descartarBytes(size_t tamanho)
{
    uint8_t discard[512];
    size_t totalDiscarded = 0;
    unsigned long timeout = millis();

    while (totalDiscarded < tamanho && millis() - timeout < 5000)
    {
        if (client.available())
        {
            size_t toRead = min((size_t)client.available(), sizeof(discard));
            toRead = min(toRead, tamanho - totalDiscarded);
            size_t n = client.read(discard, toRead);
            totalDiscarded += n;
            timeout = millis();
        }
        vTaskDelay(pdMS_TO_TICKS(1));   // FIX WATCHDOG
    }
}

// A CAM termina cada foto com "\nEND_FRAME\n". Le as duas linhas para o
// proximo cabecalho START: nao vir invalido.
static void consumirFimDeQuadro()
{
    for (int i = 0; i < 2; i++)
    {
        String l = client.readStringUntil('\n');
        l.trim();
        if (l == "END_FRAME") return;
    }
}

// Corpo do antigo receiveImage().
//   checarNova = true : comportamento antigo (sequencia 0-9, so aceita foto nova)
//   checarNova = false: baixa exatamente GET:<idx>
static bool baixarImagem(int idx, bool checarNova)
{
    monitorConnection();

    if (!client.connected())
        return false;

    client.printf("GET:%d\n", idx);
    client.flush();

    unsigned long startMs = millis();

    while (!client.available())
    {
        if (millis() - startMs > 5000)
        {
            Serial.println("servidor nao respondeu (timeout)");
            client.stop();
            conectedOnce = false;
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(1));   // FIX WATCHDOG
    }

    String header = client.readStringUntil('\n');
    header.trim();

    if (!header.startsWith("START:"))
    {
        Serial.println("header invalido recebido");
        client.stop();
        return false;
    }

    // START:<tipo>:<?>:<tamanho>:<totalIndex>:<tempo>
    int partsFound = 0;
    String parts[6];
    int lastPos = 0;

    for (int i = 0; i < (int)header.length() && partsFound < 6; i++)
    {
        if (header[i] == ':' || i == (int)header.length() - 1)
        {
            parts[partsFound++] = header.substring(lastPos, (i == (int)header.length() - 1) ? i + 1 : i);
            lastPos = i + 1;
        }
    }

    if (partsFound < 6)
    {
        client.stop();
        return false;
    }

    String currentTipoFigura = parts[1];
    currentImgSize     = (size_t)parts[3].toInt();
    currentTotalIndex  = (uint32_t)parts[4].toInt();
    currentMissionTime = parts[5].toFloat();

    if (checarNova)
    {
        bool isNewImage = (currentTotalIndex > lastTotalIndex) || (lastTotalIndex == 0xFFFFFFFF);

        if (lastTotalIndex != 0xFFFFFFFF && lastTotalIndex >= 100 &&
            currentTotalIndex < (lastTotalIndex - 100))
        {
            isNewImage = true;
        }

        if (!isNewImage)
        {
            descartarBytes(currentImgSize);
            consumirFimDeQuadro();
            return false;
        }
    }

    if (currentImgSize > MAX_IMG_SIZE)
    {
        Serial.println("imagem maior que buffer, descartando");
        descartarBytes(currentImgSize);
        consumirFimDeQuadro();
        return false;
    }

    const uint8_t SYNC_START[] = { 0xAA, 0xBB, 0xCC, 0xDD };
    const uint8_t SYNC_END[]   = { 0xEE, 0xFF };

    String logPayload = "TIPO:" + currentTipoFigura +
                        ", ID:" + String(idx) +
                        ", TOT:" + String(currentTotalIndex) +
                        ", T:" + String(currentMissionTime, 3);

    uint32_t logSize = logPayload.length();

    Serial.write(SYNC_START, 4);
    Serial.write(0x01);
    Serial.write((uint8_t*)&logSize, 4);
    Serial.print(logPayload);
    Serial.write(SYNC_END, 2);

    Serial.write(SYNC_START, 4);
    Serial.write(0x02);
    uint32_t imgSize32 = (uint32_t)currentImgSize;
    Serial.write((uint8_t*)&imgSize32, 4);

    size_t totalRead = 0;
    uint8_t buffer[1024];
    unsigned long downloadStart = millis();

    while (totalRead < currentImgSize && (millis() - downloadStart < 10000))
    {
        if (client.available())
        {
            size_t bytesToRead = min((size_t)client.available(), currentImgSize - totalRead);
            if (bytesToRead > sizeof(buffer)) bytesToRead = sizeof(buffer);

            size_t n = client.read(buffer, bytesToRead);

            Serial.write(buffer, n);

            // Guarda tambem no buffer local, para uso pelo LoRa (taskTelemetria)
            memcpy(fb_buf + totalRead, buffer, n);

            totalRead += n;
            downloadStart = millis();
        }
        vTaskDelay(pdMS_TO_TICKS(1));   // FIX WATCHDOG
    }

    Serial.write(SYNC_END, 2);

    if (totalRead == currentImgSize)
    {
        consumirFimDeQuadro();

        lastTotalIndex = currentTotalIndex;
        if (checarNova)
            nextIndexToRequest = (nextIndexToRequest + 1) % 10;

        Serial.println("download de imagem concluido");
        return true;
    }

    Serial.println("download incompleto");
    client.stop();
    return false;
}

// Comportamento antigo: sequencia cega 0-9 (a taskWiFi nao usa mais)
bool receiveImage()
{
    return baixarImagem(nextIndexToRequest, true);
}

// Baixa exatamente a foto <idx> (GET:<idx>), sem a logica de "foto nova"
bool receiveImageIndex(int idx)
{
    return baixarImagem(idx, false);
}

//Buffer de Image
uint8_t* getImageBuffer()
{
    return fb_buf;
}

uint32_t getImageSize()
{
    return (uint32_t)currentImgSize;
}

// ===========================================================================
// Missao secundaria: UDP da CAM, veredito do Orange e decisao da foto
// ===========================================================================
struct Deteccao {
    double lat;
    double lon;
    int    photoIndex;
};

static Deteccao filaDet[M2_FILA_DETECCOES];
static uint8_t  filaIni = 0, filaN = 0;
static int      ultimoIndiceRecebido = -1;   // ignora photo_index repetido

// Veredito vindo da taskTelemetria (outra task): protegido por spinlock
static portMUX_TYPE m2Mux = portMUX_INITIALIZER_UNLOCKED;
static volatile int      m2VerIndice = -1;
static volatile Veredito m2VerValor  = V_INDETERMINADO;

// Chamada pela opVeredito() (telemetria.cpp) nos tres desfechos do QRY
void m2Veredito(int photoIndex, Veredito v)
{
    if (photoIndex < 0) return;            // QRY manual (OP,QRY,...): sem foto associada
    portENTER_CRITICAL(&m2Mux);
    m2VerValor  = v;
    m2VerIndice = photoIndex;
    portEXIT_CRITICAL(&m2Mux);
}

static bool lerVeredito(int photoIndex, Veredito* out)
{
    bool ok = false;
    portENTER_CRITICAL(&m2Mux);
    if (m2VerIndice == photoIndex) { *out = m2VerValor; ok = true; }
    portEXIT_CRITICAL(&m2Mux);
    return ok;
}

static void limparVeredito()
{
    portENTER_CRITICAL(&m2Mux);
    m2VerIndice = -1;
    portEXIT_CRITICAL(&m2Mux);
}

// Extrai o numero que vem depois de "chave": num JSON simples
static bool jsonNumero(const char* json, const char* chave, double* out)
{
    char padrao[32];
    snprintf(padrao, sizeof(padrao), "\"%s\"", chave);
    const char* p = strstr(json, padrao);
    if (!p) return false;
    p = strchr(p + strlen(padrao), ':');
    if (!p) return false;
    p++;
    while (*p == ' ') p++;
    char* fim;
    double v = strtod(p, &fim);
    if (fim == p) return false;
    *out = v;
    return true;
}

// Le os pacotes UDP da CAM: {"lat":..,"lon":..,"photo_index":..,"photo_size":..}
static void pollUdp()
{
    if (WiFi.status() != WL_CONNECTED) return;

    if (!udpAberto)
    {
        udpAberto = udp.begin(ADSB_UDP_PORT);
        if (udpAberto) Serial.printf("[M2] ouvindo UDP %d\n", ADSB_UDP_PORT);
        return;
    }

    int tam;
    while ((tam = udp.parsePacket()) > 0)
    {
        char buf[160];
        int n = udp.read(buf, sizeof(buf) - 1);
        if (n <= 0) continue;
        buf[n] = 0;

        double lat, lon, idxD;
        if (!jsonNumero(buf, "lat", &lat) || !jsonNumero(buf, "lon", &lon) ||
            !jsonNumero(buf, "photo_index", &idxD))
        {
            Serial.printf("[M2] UDP invalido: %s\n", buf);
            continue;
        }

        int idx = (int)idxD;
        if (idx == ultimoIndiceRecebido) continue;   // mesma foto anunciada de novo
        ultimoIndiceRecebido = idx;

        if (filaN >= M2_FILA_DETECCOES)
        {
            Serial.printf("[M2] fila cheia, deteccao da foto %d perdida\n", idx);
            continue;
        }

        uint8_t pos = (filaIni + filaN) % M2_FILA_DETECCOES;
        filaDet[pos].lat = lat;
        filaDet[pos].lon = lon;
        filaDet[pos].photoIndex = idx;
        filaN++;

        Serial.printf("[M2] deteccao: foto %d lat=%.5f lon=%.5f\n", idx, lat, lon);
    }
}

// ===========================================================================
// Task Wi-Fi: uma deteccao por vez
// ===========================================================================
void taskWiFi(void *parameter)
{
    setupNetwork();

    bool          ativa = false;
    Deteccao      det;
    bool          fotoOk = false;
    uint8_t       tentativasFoto = 0;
    unsigned long proxTentativaFoto = 0;
    unsigned long inicioDet = 0;

    while (true)
    {
        monitorConnection();
        pollUdp();

        unsigned long agora = millis();

        // 1) Livre? pega a proxima deteccao e pergunta ao Orange
        //    (so quando a foto anterior ja saiu: fb_buf e compartilhado com o LoRa)
        if (!ativa && filaN > 0 && !imageReady && !telemetriaIsImageSending())
        {
            Deteccao prox = filaDet[filaIni];

            limparVeredito();
            if (opConsultarAeronave(prox.lat, prox.lon, nullptr, prox.photoIndex))
            {
                det = prox;
                filaIni = (filaIni + 1) % M2_FILA_DETECCOES;
                filaN--;

                ativa = true;
                fotoOk = false;
                tentativasFoto = 0;
                proxTentativaFoto = agora;
                inicioDet = agora;
            }
            // senao: fila de QRY cheia, tenta de novo na proxima volta
        }

        if (ativa)
        {
            // 2) Baixa a foto (ate M2_TENTATIVAS_FOTO tentativas)
            if (!fotoOk && tentativasFoto < M2_TENTATIVAS_FOTO &&
                (long)(agora - proxTentativaFoto) >= 0)
            {
                tentativasFoto++;
                fotoOk = receiveImageIndex(det.photoIndex);
                if (!fotoOk)
                {
                    Serial.printf("[M2] foto %d: falha no download (%d/%d)\n",
                                  det.photoIndex, tentativasFoto, M2_TENTATIVAS_FOTO);
                    proxTentativaFoto = millis() + M2_INTERVALO_FOTO_MS;
                }
            }

            bool fotoDecidida = fotoOk || tentativasFoto >= M2_TENTATIVAS_FOTO;
            Veredito v;
            bool temVeredito = lerVeredito(det.photoIndex, &v);
            bool estourou = (millis() - inicioDet) >= M2_TIMEOUT_MS;

            if (estourou && !temVeredito)
            {
                v = V_INDETERMINADO;
                temVeredito = true;
                Serial.printf("[M2] foto %d: timeout sem veredito\n", det.photoIndex);
            }

            // 3) Foto + veredito: decide
            if ((temVeredito && fotoDecidida) || estourou)
            {
                bool enviar = (v == V_CLANDESTINA) ||
                              (v == V_INDETERMINADO && M2_ENVIAR_SE_INDETERMINADO);

                if (!fotoOk)
                {
                    Serial.printf("[M2] foto %d: sem foto, nada a enviar\n", det.photoIndex);
                }
                else if (enviar)
                {
                    Serial.printf("[M2] foto %d: enviando para a base\n", det.photoIndex);
                    imageReady = true;              // taskTelemetria inicia o envio
                    lastProcessTime = millis();
                }
                else
                {
                    Serial.printf("[M2] foto %d: autorizada, descartada\n", det.photoIndex);
                }

                limparVeredito();
                ativa = false;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));      // le o UDP a cada 50 ms
    }
}