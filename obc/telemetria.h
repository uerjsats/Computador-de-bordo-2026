#ifndef TELEMETRIA_H
#define TELEMETRIA_H

#include <Arduino.h>
#include <RadioLib.h>
#include <stdint.h>
#include <string.h>
#include <HardwareSerial.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define START_BYTE   0x7E

// Tipos de pacote — devem ser iguais no OBC (mainV1.4) e na estação de solo (LoRaRX)
#define TYPE_SENSOR  0x01
#define TYPE_GPS     0x02
#define TYPE_GYRO    0x03
#define TYPE_IMAGE   0x10
#define TYPE_DEBUG   0x20
#define TYPE_COMMAND 0x30
#define TYPE_STRING  0x40

// Endereços
#define ADDR_GROUND  0x01
#define ADDR_OBC     0x02

// Tamanho mínimo de cabeçalho: START_BYTE + TYPE + SRC + DST
#define HEADER_SIZE  4

#define BUFFER_SIZE  256

// Comprimento dos campos-string preenchidos pelas placas escravas.
// Vazio ("") quando a escrava ainda nao respondeu (o RX imprime em branco).
#define SLAVE_STR_LEN  16
#define CTRL_STR_LEN   64


// --- Suprimento de energia (INA) — serial simulada ---
#define SUP_RX_PIN          19        // Heltec RX  <-  TX da placa de energia
#define SUP_TX_PIN          20        // Heltec TX  ->  RX da placa de energia
#define SUP_BAUD            9600
#define SUP_PEDIDO          "REQ_INA" // pedido enviado a placa de energia
#define SUP_INTERVALO_MS    2000UL    // pede o INA a cada 2 s
#define SUP_LINHA_MAX       128

#define OP_SERIAL_RX_PIN   6
#define OP_SERIAL_TX_PIN   7
#define OP_BAUD            115200   // tem que ser igual a Config.BAUD do adsb_cubesat.py

// --- Protocolo ---
#define OP_PREFIXO         "2:"     // endereco do Orange (Config.ENDERECO_ORANGE no .py)
#define OP_CHECKSUM_TX     1        // 1 = comandos saem com "*XX" (o .py valida quando vem)

#define OP_AUTO_START         0
#define OP_AUTO_ATRASO_MS     10000UL   // espera apos ligar o OBC antes da 1a tentativa
#define OP_AUTO_REINTENTO_MS  5000UL    // intervalo entre tentativas (PING / START)
#define OP_AUTO_DURACAO_S     600       // duracao da missao enviada no START (0 = sem limite)
#define OP_AUTO_REINICIAR     0         // 1 = ao fim dos 10 min (EVT,DONE) inicia outra missao

// Orange nao tem RTC. Se > 0, o OBC envia TIME = este epoch + uptime do OBC.
// Ex.: 1761000000UL. Deixe 0 para nao enviar (ou use "OP,TIME,<epoch>" pelo solo).
#define OP_TIME_EPOCH_INICIAL 0UL

// --- Supervisao ---
#define OP_STATUS_INTERVALO_MS 30000UL  // com missao rodando, pede STATUS a cada 30 s (e repassa ao solo)
#define OP_SILENCIO_MS         90000UL  // sem nenhuma linha valida do Orange por 90 s => volta a SEM_CONTATO

// --- Missao secundaria (QRY) ---
#define OP_QRY_TIMEOUT_MS   3000UL      // espera pelo QRES
#define OP_QRY_TENTATIVAS   3           // depois disso o veredito vira INDETERMINADO

// --- LoRa ---
#define OP_AC_AGRUPAR  1                // 1 = junta varios "AC" em um unico pacote LoRa (separados por '\n')

// --- Debug ---
#define OP_CONSOLE_USB 1                // 1 = aceita "OP,<cmd>" digitado no Monitor Serial (USB)
#define OP_LOG_AC      1                // 1 = imprime cada AC no Monitor Serial
#define OP_DEBUG_RAW   0                // 1 = imprime cada byte vindo do Orange (hex se nao imprimivel)

#define PACKET_SIZE     180
#define MAX_IMAGE_SIZE  50000

#define RX_WINDOW_MS  1000      // tempo so ouvindo depois de cada TX / RX

#define OP_LINHA_MAX 200
#define OP_LOTE_MAX  200      // bytes de payload LoRa por lote de AC (+4 de cabecalho < 256)
#define OPLOG(...)   Serial.printf(__VA_ARGS__)

#define CTRL_BAUD         9600
#define CTRL_RX_PIN       33
#define CTRL_TX_PIN       34
#define CTRL_TIMEOUT_MS   1000  // tempo maximo esperando a resposta do controle

#define RF_FREQUENCY 915.0
#define TX_OUTPUT_POWER 20
#define LORA_BANDWIDTH 125.0
#define LORA_SPREADING_FACTOR 7
#define LORA_CODINGRATE 5

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



extern FilaStr<16, 120> prioFila;
extern FilaStr<24, 96> acFila;
extern FilaStr<4, SUP_LINHA_MAX> supFila;

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

#pragma pack(push, 1)
struct sensorsData {
    // 1. Tempo (s)
    uint32_t seconds;

    // 2. Temperatura ambiente (°C × 100)
    int16_t temperatura;

    // 3. Umidade (% × 100)
    int16_t umidade;

    // 4. Altitude (m × 10)
    int16_t altitude;

    // 5. Pressão (Pa)
    uint32_t pressao;

    // 6. Latitude (graus × 10^7)
    int32_t latitude;

    // 7. Longitude (graus × 10^7)
    int32_t longitude;

    // 8. Número de satélites
    uint8_t sats;

    // 9. Roll (graus × 100)
    int16_t roll;

    // 10. Pitch (graus × 100)
    int16_t pitch;

    // 11. Yaw (graus × 100)
    int16_t yaw;
    
    // 13. Temperatura da bateria 1 (°C × 100)
    int16_t temperaturaBateria1;

    // 14. Temperatura da bateria 2 (°C × 100)
    int16_t temperaturaBateria2;

    // 15. Tensão de alimentação (V × 1000)
    uint16_t tensao;

    // 16. Corrente elétrica (A × 1000)
    int16_t corrente;
};
#pragma pack(pop)

// Constrói cabeçalho de 4 bytes no buffer. Retorna 4 (próximo offset).
inline uint8_t buildHeader(uint8_t* buf, uint8_t type, uint8_t src, uint8_t dst) {
    buf[0] = START_BYTE;
    buf[1] = type;
    buf[2] = src;
    buf[3] = dst;
    return HEADER_SIZE;
}

// Valida cabeçalho mínimo. Retorna true se válido.
inline bool validateHeader(const uint8_t* buf, uint16_t size) {
    if (size < HEADER_SIZE) return false;
    if (buf[0] != START_BYTE) return false;
    return true;
}

// Extrai tipo do pacote
inline uint8_t packetType(const uint8_t* buf) {
    return buf[1];
}

// Extrai endereço de origem
inline uint8_t packetSrc(const uint8_t* buf) {
    return buf[2];
}

// Extrai endereço de destino
inline uint8_t packetDst(const uint8_t* buf) {
    return buf[3];
}

// Payload começa no byte 4
inline const uint8_t* packetPayload(const uint8_t* buf) {
    return &buf[HEADER_SIZE];
}

inline uint16_t packetPayloadSize(uint16_t totalSize) {
    return (totalSize > HEADER_SIZE) ? (totalSize - HEADER_SIZE) : 0;
}

// Parse sensor data from payload (already stripped of header)
inline bool parseSensorData(const uint8_t* payload, uint16_t payloadSize, sensorsData* out) {
    if (payloadSize < sizeof(sensorsData)) return false;
    memcpy(out, payload, sizeof(sensorsData));
    return true;
}

void telemetriaInit(uint8_t myAddress, uint8_t destAddress);
void telemetriaProcess();

bool telemetriaSendPacket(uint8_t* data, uint16_t size, uint8_t type);
bool telemetriaSendPacket(const char* payload, uint8_t type);

void telemetriaOnPacketReceived(void (*callback)(uint8_t*, uint16_t));

// Envio de imagens (fragmentado em chunks)
void telemetriaSendImageRaw(uint8_t* data, uint32_t size);
void telemetriaSendImage(uint8_t* data, uint16_t size);
void telemetriaSendImageChunk();
bool telemetriaIsImageSending();

bool telemetriaIsIdle();
void telemetriaSetTxInterval(unsigned long interval);

bool telemetriaSendString(const char* texto);
bool telemetriaSendString(const String& texto);

// ---------------------------------------------------------------------
//  API da missao Orange Pi
// ---------------------------------------------------------------------

// Missao secundaria: pergunta ao Orange se a aeronave detectada pelo ESP32 e
// AUTORIZADA (nao clandestina). Pode ser chamada de QUALQUER task (ex.: a que
// recebe o pacote Wi-Fi/TCP do ESP32). Nao bloqueia: so enfileira.
// O veredito sai sozinho para o solo via LoRa:
//   VEREDITO,NAO_CLANDESTINA,<icao>,<callsign>,<lat>,<lon>
//   VEREDITO,CLANDESTINA,<lat>,<lon>
//   VEREDITO,INDETERMINADO,<lat>,<lon>     (Orange nao respondeu)
// Retorna false se a fila esta cheia ou a telemetria ainda nao iniciou.
bool opConsultarAeronave(double lat, double lon, const char* icao24 = nullptr);

// Executa um comando "de solo" (sem o prefixo "OP,"). Ex.: "START,60", "STOP",
// "QRY,-22.88,-43.30", "AUTO,0", "ESTADO". Chamado pelo callback LoRa e pelo console USB.
void opComandoSolo(const String& cmd);

void taskTelemetria(void *parameter);

#endif