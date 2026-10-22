#include "telemetria.h"
#include "networkManager.h"
#include <strings.h>

#define RF_FREQUENCY 915.0
#define TX_OUTPUT_POWER 20
#define LORA_BANDWIDTH 125.0
#define LORA_SPREADING_FACTOR 7
#define LORA_CODINGRATE 5

// Acumulador de linha parcial da serial de controle (nao bloqueante)
static String accC;

HardwareSerial controleSerial(2);
HardwareSerial orangeSerial(1);     // UART1 <-> Orange Pi

#define PACKET_SIZE     180
#define MAX_IMAGE_SIZE  50000

extern QueueHandle_t filaTelemetria;

static SX1262 radio = new Module(8, 14, 12, 13);

static volatile bool receivedFlag = false;

static uint8_t myAddress;
static uint8_t destAddress;

static bool idle = true;
static unsigned long txInterval = 300;
static unsigned long lastTxTime = 0;

static uint8_t rxBuffer[BUFFER_SIZE];
static uint8_t txBuffer[BUFFER_SIZE];

static void (*packetCallback)(uint8_t*, uint16_t) = nullptr;
struct ImageTransaction {
    uint8_t* data  = nullptr;
    uint16_t size  = 0;
    uint16_t index = 0;
    uint16_t total = 0;
    bool ativo     = false;
};

static uint8_t binaryBuffer[MAX_IMAGE_SIZE];
static uint8_t rleBuffer[MAX_IMAGE_SIZE];
static ImageTransaction imgTx;

static void setFlag()
{
    receivedFlag = true;
}

static void handleReceived(uint8_t* payload, uint16_t size)
{
    if (!validateHeader(payload, size)) {
        return;
    }

    uint8_t src = packetSrc(payload);
    uint8_t dst = packetDst(payload);

    // So aceita pacotes enderecados a este dispositivo e vindos do par esperado
    if (dst != myAddress || src != destAddress) {
        return;
    }

    // Repassa o pacote INTEIRO (com cabecalho) para o callback.
    // O callback usa packetType()/packetPayload()/packetPayloadSize() de protocol.h
    // para interpretar.
    if (packetCallback)
        packetCallback(payload, size);
}

static void binarize(uint8_t* input, uint8_t* output, int size, uint8_t threshold)
{
    for (int i = 0; i < size; i++) {
        output[i] = (input[i] > threshold) ? 1 : 0;
    }
}

static int encodeRLE(uint8_t* input, int size, uint8_t* output)
{
    int outIndex = 0;
    uint8_t current = input[0];
    uint8_t count = 1;

    for (int i = 1; i < size; i++) {
        if (input[i] == current && count < 255) {
            count++;
        } else {
            output[outIndex++] = count;
            output[outIndex++] = current;
            current = input[i];
            count = 1;
        }
    }

    output[outIndex++] = count;
    output[outIndex++] = current;

    return outIndex;
}

void telemetriaInit(uint8_t myAddr, uint8_t destAddr)
{
    myAddress = myAddr;
    destAddress = destAddr;

    int state = radio.begin(RF_FREQUENCY, LORA_BANDWIDTH, LORA_SPREADING_FACTOR, LORA_CODINGRATE);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[LORA] ERRO radio.begin: %d\n", state);
        return;
    }

    radio.setOutputPower(TX_OUTPUT_POWER);
    radio.setPacketReceivedAction(setFlag);
    radio.startReceive();
}

void telemetriaProcess()
{
    if (receivedFlag) {
        receivedFlag = false;

        if (!idle) {
            radio.finishTransmit();
            idle = true;
            radio.startReceive();
            return;
        }

        int len = radio.getPacketLength();

        if (len > 0 && len <= BUFFER_SIZE) {
            int state = radio.readData(rxBuffer, len);
            if (state == RADIOLIB_ERR_NONE) {
                handleReceived(rxBuffer, len);
            }
        }

        radio.startReceive();
    }
}

void telemetriaSendPacket(uint8_t* data, uint16_t size, uint8_t type)
{
    if (!idle) return;
    if (millis() - lastTxTime < txInterval) return;
    if ((uint32_t)size + HEADER_SIZE > BUFFER_SIZE) return;

    uint8_t idx = buildHeader(txBuffer, type, myAddress, destAddress);
    memcpy(&txBuffer[idx], data, size);

    idle = false;
    radio.startTransmit(txBuffer, size + idx);
    lastTxTime = millis();
}

void telemetriaSendPacket(const char* payload, uint8_t type)
{
    telemetriaSendPacket((uint8_t*)payload, strlen(payload), type);
}

void telemetriaOnPacketReceived(void (*callback)(uint8_t*, uint16_t))
{
    packetCallback = callback;
}

void telemetriaSendImageRaw(uint8_t* data, uint32_t size)
{
    if (data == nullptr || size == 0) return;

    if (size > MAX_IMAGE_SIZE) {
        return;
    }

    telemetriaSendImage(data, size);
}

void telemetriaSendImage(uint8_t* data, uint16_t size)
{
    if (imgTx.ativo) return;

    imgTx.data = data;
    imgTx.size = size;
    imgTx.index = 0;
    imgTx.total = (size + PACKET_SIZE - 1) / PACKET_SIZE;
    imgTx.ativo = true;

}

void telemetriaSendImageChunk()
{
    if (!imgTx.ativo) return;
    if (!idle) return;
    if (millis() - lastTxTime < txInterval) return;

    if (imgTx.index >= imgTx.total) {
        imgTx.ativo = false;
        return;
    }

    int start = imgTx.index * PACKET_SIZE;
    int remaining = imgTx.size - start;
    int len = (remaining > PACKET_SIZE) ? PACKET_SIZE : remaining;

    uint8_t packet[PACKET_SIZE + 7];

    uint8_t idx = buildHeader(packet, TYPE_IMAGE, myAddress, destAddress);
    packet[idx++] = (uint8_t)imgTx.index;
    packet[idx++] = (uint8_t)imgTx.total;
    packet[idx++] = (uint8_t)len;

    memcpy(packet + idx, imgTx.data + start, len);

    idle = false;

    int state = radio.startTransmit(packet, len + idx);

    if (state != RADIOLIB_ERR_NONE) {
        imgTx.ativo = false;
        idle = true;
        return;
    }

    lastTxTime = millis();
    imgTx.index++;
}

bool telemetriaIsImageSending()
{
    return imgTx.ativo;
}

bool telemetriaIsIdle()
{
    return idle;
}

void telemetriaSetTxInterval(unsigned long interval)
{
    txInterval = interval;
}

// =====================================================================
//  MISSAO ADS-B — ORANGE PI
//  Protocolo (igual ao adsb_cubesat.py):
//    OBC -> Orange:  2:COMANDO[,arg,...][*XX]
//    Orange -> OBC:  2:TIPO[,campo,...][*XX]
//    XX = XOR hex dos caracteres depois de "2:" (opcional; validado se vier)
// =====================================================================

#define OP_LINHA_MAX 200
#define OP_LOTE_MAX  200      // bytes de payload LoRa por lote de AC (+4 de cabecalho < 256)
#define OPLOG(...)   Serial.printf(__VA_ARGS__)

// Fila circular de strings (acessada so pela taskTelemetria)
template <uint8_t N, size_t L>
struct FilaStr {
    char d[N][L];
    uint8_t h = 0, t = 0, n = 0;
    uint32_t perdidos = 0;

    void push(const char* s) {
        if (n == N) { h = (h + 1) % N; n--; perdidos++; }   // cheia: descarta a mais antiga
        strncpy(d[t], s, L - 1);
        d[t][L - 1] = 0;
        t = (t + 1) % N;
        n++;
    }
    bool vazia() const { return n == 0; }
    const char* em(uint8_t i) const { return d[(h + i) % N]; }
    void pop() { if (n) { h = (h + 1) % N; n--; } }
};

static FilaStr<16, 120> prioFila;   // EVT, ACK, NAK, STATUS, VEREDITO, respostas do controle
static FilaStr<24, 96>  acFila;     // aeronaves decodificadas

enum OpEstado : uint8_t { OP_SEM_CONTATO = 0, OP_VIVO = 1, OP_EXECUTANDO = 2 };
static const char* const OP_NOMES[] = {"SEM_CONTATO", "VIVO", "EXECUTANDO"};

static OpEstado      opEstado = OP_SEM_CONTATO;
static bool          opAutoAtivo = (OP_AUTO_START != 0);
static bool          opMissaoConcluida = false;
static unsigned long opProxTentativa = 0;
static unsigned long opUltimoRx = 0;
static unsigned long opUltimoStatus = 0;
static uint32_t      opAcRx = 0, opLinhasOk = 0, opLinhasInvalidas = 0;

// Consulta QRY em andamento (uma por vez)
struct OpQry { double lat; double lon; char icao[8]; };
static QueueHandle_t filaQry = nullptr;
static OpQry         qryAtual;
static bool          qryAtiva = false;
static unsigned long qryT = 0;
static uint8_t       qryTent = 0;

static char   opAcc[OP_LINHA_MAX];
static size_t opAccLen = 0;

static void opMudar(OpEstado e)
{
    if (e == opEstado) return;
    opEstado = e;
    OPLOG("[OP] estado -> %s\n", OP_NOMES[e]);
}

// Envia um comando ao Orange: "2:CORPO[*XX]\r\n"
static void opEnviar(const char* corpo)
{
    String linha = String(OP_PREFIXO) + corpo;          // ex.: "2:PING"

#if OP_CHECKSUM_TX
    uint8_t cs = 0;
    for (const char* p = corpo; *p; p++) cs ^= (uint8_t)*p;
    char ck[4];
    snprintf(ck, sizeof(ck), "*%02X", cs);
    linha += ck;                                        // ex.: "2:PING*10"
#endif

    orangeSerial.println(linha);
    Serial.println("[OP] TX: " + linha);
}

static void opEncaminhar(const char* corpo)
{
    prioFila.push(corpo);
}

static int opDividir(char* s, char** tok, int max)
{
    int n = 0;
    if (!*s) return 0;
    tok[n++] = s;
    for (char* p = s; *p && n < max; p++) {
        if (*p == ',') { *p = 0; tok[n++] = p + 1; }
    }
    return n;
}

static void opAoFicarVivo()
{
    opMudar(OP_VIVO);
    opProxTentativa = millis();     // manda o START logo em seguida
#if OP_TIME_EPOCH_INICIAL > 0
    char b[40];
    snprintf(b, sizeof(b), "TIME,%lu", (unsigned long)(OP_TIME_EPOCH_INICIAL + millis() / 1000UL));
    opEnviar(b);
#endif
}

static void opEnviarQry(unsigned long now)
{
    char b[64];
    if (qryAtual.icao[0])
        snprintf(b, sizeof(b), "QRY,%.5f,%.5f,%s", qryAtual.lat, qryAtual.lon, qryAtual.icao);
    else
        snprintf(b, sizeof(b), "QRY,%.5f,%.5f", qryAtual.lat, qryAtual.lon);
    opEnviar(b);
    qryT = now;
    qryTent++;
}

static void opVeredito(const char* tipo, const char* extra)
{
    char b[120];
    snprintf(b, sizeof(b), "VEREDITO,%s,%s%.5f,%.5f", tipo, extra, qryAtual.lat, qryAtual.lon);
    OPLOG("[OP] %s\n", b);
    prioFila.push(b);
}

static void opPublicarEstado()
{
    char b[120];
    snprintf(b, sizeof(b), "OPSTATE,%s,%d,%d,%lu,%lu,%lu,%lu",
             OP_NOMES[opEstado], opAutoAtivo ? 1 : 0, opMissaoConcluida ? 1 : 0,
             (unsigned long)opAcRx, (unsigned long)acFila.perdidos,
             (unsigned long)opLinhasOk, (unsigned long)opLinhasInvalidas);
    OPLOG("[OP] %s\n", b);
    prioFila.push(b);
}

// Trata uma linha completa vinda do Orange ("2:TIPO,campos[*XX]").
static void opTratarLinha(const char* bruta)
{
    // Procura o endereco "2:" (tolera lixo antes, ex.: boot do U-Boot)
    const char* ini = strstr(bruta, OP_PREFIXO);
    if (!ini) { opLinhasInvalidas++; return; }          // lixo ou outro subsistema
    ini += strlen(OP_PREFIXO);

    char corpo[OP_LINHA_MAX];
    strncpy(corpo, ini, sizeof(corpo) - 1);
    corpo[sizeof(corpo) - 1] = 0;

    // Tira espacos no fim
    size_t lc = strlen(corpo);
    while (lc && corpo[lc - 1] == ' ') corpo[--lc] = 0;

    // Checksum opcional: se vier "*XX", tem que bater
    char* ast = strrchr(corpo, '*');
    if (ast) {
        uint8_t cs = 0;
        for (char* p = corpo; p < ast; p++) cs ^= (uint8_t)*p;
        char* fim = nullptr;
        long rx = strtol(ast + 1, &fim, 16);
        if (fim == ast + 1 || *fim != 0 || (uint8_t)rx != cs) {
            opLinhasInvalidas++;
            OPLOG("[OP] checksum invalido, ignorada\n");
            return;
        }
        *ast = 0;                                       // corpo sem o "*XX"
    }
    if (!corpo[0]) return;

    opLinhasOk++;
    opUltimoRx = millis();
    if (opEstado == OP_SEM_CONTATO) opAoFicarVivo();   // qualquer linha valida = Orange vivo (envia TIME se configurado)

    char w[OP_LINHA_MAX];
    strcpy(w, corpo);
    char* t[16];
    int nt = opDividir(w, t, 16);
    if (nt == 0) return;
    #define T(i) (nt > (i) ? t[i] : "")

    // ---- AC: aeronave decodificada ----
    if (!strcmp(t[0], "AC")) {
        opAcRx++;
        if (opEstado != OP_EXECUTANDO) opMudar(OP_EXECUTANDO);   // AC prova que esta rodando
        acFila.push(corpo);
#if OP_LOG_AC
        OPLOG("[OP] RX: %s\n", corpo);
#endif
        return;
    }

    OPLOG("[OP] RX: %s\n", corpo);

    // ---- EVT ----
    if (!strcmp(t[0], "EVT")) {
        if (!strcmp(T(1), "BOOT")) {
            opMissaoConcluida = false;      // Orange reiniciou (ex.: queda de energia): recomeca
            opAoFicarVivo();
        } else if (!strcmp(T(1), "DONE")) {
            opMudar(OP_VIVO);
            opMissaoConcluida = (OP_AUTO_REINICIAR == 0);
            opProxTentativa = millis();
        }
        opEncaminhar(corpo);
        return;
    }

    // ---- ACK ----
    if (!strcmp(t[0], "ACK")) {
        if (!strcmp(T(1), "PING")) {
            // contato ja tratado acima (opAoFicarVivo)
        } else if (!strcmp(T(1), "START")) {
            opMudar(OP_EXECUTANDO);
            opUltimoStatus = millis();
        } else if (!strcmp(T(1), "STOP")) {
            opMudar(OP_VIVO);
        }
        opEncaminhar(corpo);
        return;
    }

    // ---- NAK ----
    if (!strcmp(t[0], "NAK")) {
        if (!strcmp(T(1), "START") && !strcmp(T(2), "JA_EM_EXECUCAO")) {
            opMudar(OP_EXECUTANDO);
            opUltimoStatus = millis();
        } else if (!strcmp(T(1), "STOP") && !strcmp(T(2), "OCIOSO")) {
            opMudar(OP_VIVO);
        }
        opEncaminhar(corpo);
        return;
    }

    // ---- STATUS ----
    if (!strcmp(t[0], "STATUS")) {
        if (!strcasecmp(T(1), "RUNNING"))      { if (opEstado != OP_EXECUTANDO) opMudar(OP_EXECUTANDO); }
        else if (!strcasecmp(T(1), "IDLE"))    { if (opEstado == OP_EXECUTANDO) opMudar(OP_VIVO); }
        opEncaminhar(corpo);
        return;
    }

    // ---- QRES: resposta da missao secundaria ----
    if (!strcmp(t[0], "QRES")) {
        if (!qryAtiva) { opEncaminhar(corpo); return; }      // resposta sem consulta pendente
        if (!strcmp(T(1), "FOUND")) {
            char ex[40];
            snprintf(ex, sizeof(ex), "%s,%s,", T(2), T(3));
            opVeredito("NAO_CLANDESTINA", ex);
        } else if (!strcmp(T(1), "NONE")) {
            opVeredito("CLANDESTINA", "");
        } else {
            opEncaminhar(corpo);
        }
        qryAtiva = false;
        return;
    }

    // Outros tipos: so repassa ao solo
    opEncaminhar(corpo);
    #undef T
}

// Le a UART do Orange sem bloquear. Uma linha = tudo ate '\n' ou '\r'.
// Com OP_DEBUG_RAW, imprime cada byte recebido antes de qualquer filtro.
static void opLerOrange()
{
    int guarda = 0;
    while (orangeSerial.available() && guarda++ < 512) {
        char c = (char)orangeSerial.read();

#if OP_DEBUG_RAW
        if ((uint8_t)c >= 32 && (uint8_t)c <= 126) Serial.write(c);
        else if (c == '\n') Serial.print("<LF>\n");
        else if (c == '\r') Serial.print("<CR>");
        else Serial.printf("<%02X>", (uint8_t)c);
#endif

        if (c == '\n' || c == '\r') {
            if (opAccLen) {
                opAcc[opAccLen] = 0;
                opTratarLinha(opAcc);
                opAccLen = 0;
            }
            continue;
        }
        if (opAccLen < sizeof(opAcc) - 1) opAcc[opAccLen++] = c;
        else opAccLen = 0;      // estouro: descarta
    }
}

bool opConsultarAeronave(double lat, double lon, const char* icao24)
{
    if (!filaQry) return false;
    OpQry q;
    q.lat = lat;
    q.lon = lon;
    q.icao[0] = 0;
    if (icao24) {
        strncpy(q.icao, icao24, sizeof(q.icao) - 1);
        q.icao[sizeof(q.icao) - 1] = 0;
        for (char* p = q.icao; *p; p++) *p = toupper((unsigned char)*p);
    }
    return xQueueSend(filaQry, &q, 0) == pdPASS;
}

void opComandoSolo(const String& cmdIn)
{
    String c = cmdIn;
    c.trim();
    c.toUpperCase();
    if (c.length() == 0 || c.length() > OP_LINHA_MAX - 10) {
        prioFila.push("NAK,OP,VAZIO_OU_LONGO");
        return;
    }

    int v = c.indexOf(',');
    String nome = (v < 0) ? c : c.substring(0, v);
    String arg  = (v < 0) ? String("") : c.substring(v + 1);

    // --- comandos locais do OBC ---
    if (nome == "AUTO") {
        if (arg == "1") {
            opAutoAtivo = true;
            opMissaoConcluida = false;
            opProxTentativa = millis();
            prioFila.push("ACK,OP,AUTO,1");
        } else if (arg == "0") {
            opAutoAtivo = false;
            prioFila.push("ACK,OP,AUTO,0");
        } else {
            prioFila.push("NAK,OP,AUTO,ARG");
        }
        return;
    }
    if (nome == "ESTADO") { opPublicarEstado(); return; }

    // --- missao secundaria: passa pela fila para gerar o VEREDITO ---
    if (nome == "QRY") {
        int p1 = arg.indexOf(',');
        if (p1 < 0) { prioFila.push("NAK,OP,QRY,ARG"); return; }
        int p2 = arg.indexOf(',', p1 + 1);
        double lat = arg.substring(0, p1).toDouble();
        double lon = (p2 < 0 ? arg.substring(p1 + 1) : arg.substring(p1 + 1, p2)).toDouble();
        String icao = (p2 < 0) ? String("") : arg.substring(p2 + 1);
        if (fabs(lat) > 90.0 || fabs(lon) > 180.0) { prioFila.push("NAK,OP,QRY,FAIXA"); return; }
        if (!opConsultarAeronave(lat, lon, icao.length() ? icao.c_str() : nullptr))
            prioFila.push("NAK,OP,QRY,FILA_CHEIA");
        return;
    }

    // --- comandos repassados ao Orange ---
    if (nome == "PING" || nome == "START" || nome == "STOP" || nome == "STATUS" ||
        nome == "TIME" || nome == "MODE"  || nome == "DUMP") {
        if (nome == "STOP")  opAutoAtivo = false;           // operador mandou parar: nao reinicia sozinho
        if (nome == "START") opMissaoConcluida = false;
        opEnviar(c.c_str());
        return;
    }

    prioFila.push("NAK,OP,CMD_DESCONHECIDO");
}

#if OP_CONSOLE_USB
static String usbAcc;
static void opLerUSB()
{
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (usbAcc.length()) {
                String l = usbAcc;
                usbAcc = "";
                l.trim();
                String u = l;
                u.toUpperCase();
                if (u.startsWith("OP,")) opComandoSolo(l.substring(3));
                else Serial.println("[OP] use: OP,<comando>  ex.: OP,START,60");
            }
        } else if (usbAcc.length() < 120) {
            usbAcc += c;
        }
    }
}
#else
static void opLerUSB() {}
#endif

static void opInit()
{
    filaQry = xQueueCreate(8, sizeof(OpQry));
    orangeSerial.setRxBufferSize(1024);
    orangeSerial.begin(OP_BAUD, SERIAL_8N1, OP_SERIAL_RX_PIN, OP_SERIAL_TX_PIN);
    opProxTentativa = millis() + OP_AUTO_ATRASO_MS;
    opUltimoRx = millis();
    OPLOG("[OP] pronto. auto=%d  RX=%d TX=%d  baud=%d  prefixo=%s\n",
          OP_AUTO_START, OP_SERIAL_RX_PIN, OP_SERIAL_TX_PIN, OP_BAUD, OP_PREFIXO);
}

// Chamada a cada volta do loop da task.
static void opTick(unsigned long now)
{
    opLerUSB();
    opLerOrange();

    // ---- missao secundaria (1 consulta por vez, com timeout e retentativas) ----
    if (!qryAtiva && filaQry && xQueueReceive(filaQry, &qryAtual, 0) == pdPASS) {
        qryAtiva = true;
        qryTent = 0;
        opEnviarQry(now);
    } else if (qryAtiva && (now - qryT) >= OP_QRY_TIMEOUT_MS) {
        if (qryTent < OP_QRY_TENTATIVAS) {
            opEnviarQry(now);
        } else {
            opVeredito("INDETERMINADO", "");
            qryAtiva = false;
        }
    }

    // ---- missao autonoma: PING ate responder, depois START ----
    if (opAutoAtivo && !opMissaoConcluida && opEstado != OP_EXECUTANDO &&
        (long)(now - opProxTentativa) >= 0) {
        if (opEstado == OP_SEM_CONTATO) {
            opEnviar("PING");
        } else {
            char b[32];
            snprintf(b, sizeof(b), "START,%d", OP_AUTO_DURACAO_S);
            opEnviar(b);
        }
        opProxTentativa = now + OP_AUTO_REINTENTO_MS;
    }

    // ---- supervisao com a missao rodando ----
    if (opEstado == OP_EXECUTANDO) {
        if ((now - opUltimoStatus) >= OP_STATUS_INTERVALO_MS) {
            opEnviar("STATUS");
            opUltimoStatus = now;
        }
        if ((now - opUltimoRx) >= OP_SILENCIO_MS) {
            OPLOG("[OP] silencio do Orange\n");
            opMudar(OP_SEM_CONTATO);
            opProxTentativa = now;
            prioFila.push("EVT,OP_SILENCIO");
        }
    }
}

// ----- Despacho LoRa -----
static void opDespacharPrio(unsigned long now)
{
    if (!idle || (now - lastTxTime) < txInterval) return;
    if (prioFila.vazia()) return;
    if (telemetriaSendString(prioFila.em(0))) {
        Serial.print("[TX->BASE] ");
        Serial.println(prioFila.em(0));   // lê antes do pop
        prioFila.pop();
    }
}

static void opDespacharAC(unsigned long now)
{
    if (!idle || (now - lastTxTime) < txInterval) return;
    if (acFila.vazia()) return;

    static char lote[OP_LOTE_MAX + 1];
    size_t len = 0;
    uint8_t k = 0;
    for (uint8_t i = 0; i < acFila.n; i++) {
        const char* s = acFila.em(i);
        size_t l = strlen(s);
        size_t extra = len ? l + 1 : l;
        if (len + extra > OP_LOTE_MAX) break;
        if (len) lote[len++] = '\n';
        memcpy(lote + len, s, l);
        len += l;
        lote[len] = 0;
        k++;
#if !OP_AC_AGRUPAR
        break;
#endif
    }
    if (k && telemetriaSendString(lote)) {
        for (uint8_t i = 0; i < k; i++) acFila.pop();
    }
}

// =====================================================================

static void callbackTelemetria(uint8_t* data, uint16_t size)
{
    uint8_t type = packetType(data);
    const uint8_t* payload = packetPayload(data);
    uint16_t payloadLen = packetPayloadSize(size);

    if (type != TYPE_COMMAND) {
        return;
    }

    String cmd = "";
    for (uint16_t i = 0; i < payloadLen; i++) {
        cmd += (char)payload[i];
    }
    cmd.trim();
    Serial.println(cmd);

    // "OP,<comando>" => missao do Orange Pi. Qualquer outra coisa segue para a placa de controle.
    String up = cmd;
    up.toUpperCase();
    if (up.startsWith("OP,")) {
        opComandoSolo(cmd.substring(3));
        return;
    }

    controleSerial.println(cmd);
}


void serialInit()
{

    controleSerial.begin(9600, SERIAL_8N1, 33, 34);

}

bool telemetriaSendString(const char* texto)
{
    if (texto == nullptr) return false;

    uint16_t size = strlen(texto);

    if (size == 0) return false;
    if (!idle) return false;
    if (millis() - lastTxTime < txInterval) return false;
    if ((uint32_t)size + HEADER_SIZE > BUFFER_SIZE) return false;

    uint8_t idx = buildHeader(txBuffer, TYPE_STRING, myAddress, destAddress);
    memcpy(&txBuffer[idx], texto, size);

    idle = false;

    int state = radio.startTransmit(txBuffer, size + idx);

    if (state != RADIOLIB_ERR_NONE) {
        idle = true;
        return false;
    }

    lastTxTime = millis();
    return true;
}

bool telemetriaSendString(const String& texto)
{
    return telemetriaSendString(texto.c_str());
}

void taskTelemetria(void *parameter)
{
    sensorsData dados;
    sensorsData ultimoSensor;
    bool sensorPendente = false;

    telemetriaOnPacketReceived(callbackTelemetria);
    telemetriaInit(ADDR_OBC, ADDR_GROUND);

    unsigned long lastSensorSend = 0;

    serialInit();
    opInit();

    while (true)
    {
        telemetriaProcess();

        unsigned long now = millis();

        // Missao Orange: le serial, console USB, QRY, missao autonoma, supervisao
        opTick(now);

        // 1) Guarda o dado de sensores mais recente
        if (xQueueReceive(filaTelemetria, &dados, 0) == pdPASS)
        {
            ultimoSensor = dados;
            sensorPendente = true;
        }

        // 2) Serial de controle (nao bloqueante). A resposta vai para a fila prioritaria.
        while (controleSerial.available())
        {
            char c = (char)controleSerial.read();
            if (c == '\n')
            {
                accC.trim();
                if (accC.length())
                {
                    Serial.print("[CTRL]: ");
                    Serial.println(accC);
                    prioFila.push(accC.c_str());
                }
                accC = "";
            }
            else if (c != '\r' && accC.length() < CTRL_STR_LEN * 2)
            {
                accC += c;
            }
        }

        // 3) Imagem pronta -> inicia transmissao
        if (imageReady && !telemetriaIsImageSending())
        {
            uint8_t* imgBuf = getImageBuffer();
            uint32_t imgSize = getImageSize();

            if (imgBuf != nullptr && imgSize > 0)
            {
                telemetriaSendImageRaw(imgBuf, imgSize);
            }

            imageReady = false;
        }

        // 4) Despacho LoRa. Prioridade: mensagens (EVT/ACK/STATUS/VEREDITO) > imagem > sensores > AC
        if (!prioFila.vazia())
        {
            opDespacharPrio(now);
        }
        else if (telemetriaIsImageSending())
        {
            telemetriaSendImageChunk();
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        else if (sensorPendente && telemetriaIsIdle() && now - lastSensorSend >= 2000)
        {
            telemetriaSendPacket((uint8_t*)&ultimoSensor, sizeof(ultimoSensor), TYPE_SENSOR);
            lastSensorSend = now;
            sensorPendente = false;
        }
        else
        {
            opDespacharAC(now);
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
