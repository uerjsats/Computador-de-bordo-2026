#ifndef TELEMETRIA_H
#define TELEMETRIA_H

#include <Arduino.h>
#include <RadioLib.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#define START_BYTE   0x7E

// Tipos de pacote — devem ser iguais no OBC (mainV1.4) e na estação de solo (LoRaRX)
#define TYPE_SENSOR  0x01
#define TYPE_GPS     0x02
#define TYPE_GYRO    0x03
#define TYPE_RESPOST 0x04
#define TYPE_COMMAND 0x30

// Endereços
#define ADDR_GROUND  0x01
#define ADDR_OBC     0x02

// Tamanho mínimo de cabeçalho: START_BYTE + TYPE + SRC + DST
#define HEADER_SIZE  4

#define BUFFER_SIZE  256

// Estrutura de dados de sensores
// Comprimento dos campos-string preenchidos pelas placas escravas.
// Vazio ("") quando a escrava ainda nao respondeu (o RX imprime em branco).
#define SLAVE_STR_LEN  32
#define CTRL_STR_LEN   64

#pragma pack(push, 1)
struct sensorsData {
    // 1. Tempo (s)
    uint32_t seconds;

    // 2. Temperatura (°C × 100)
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

    // 8. Número de Satélites
    uint8_t sats;

    // 9.  Roll  / Giro X (graus × 100)
    int16_t roll;

    // 10. Pitch / Giro Y (graus × 100)
    int16_t pitch;

    // 11. Yaw   / Giro Z (graus × 100)
    int16_t yaw;

};

// Resposta completa enviada no downlink (TYPE_RESPOST):
// telemetria + resposta de controle de atitude (placa escrava addr 3, = cData).
struct respost {
    sensorsData sensor;             // Telemetria (suprimento embutido como strings)
    char controle[CTRL_STR_LEN];    // Resposta de controle de atitude (cData)
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

// Parse respost completo (telemetria + resposta de controle) do payload.
inline bool parseRespost(const uint8_t* payload, uint16_t payloadSize, respost* out) {
    if (payloadSize < sizeof(respost)) return false;
    memcpy(out, payload, sizeof(respost));
    return true;
}

void telemetriaInit(uint8_t myAddress, uint8_t destAddress);
void telemetriaProcess();

void telemetriaSendPacket(uint8_t* data, uint16_t size, uint8_t type);
void telemetriaSendPacket(const char* payload, uint8_t type);

void telemetriaOnPacketReceived(void (*callback)(uint8_t*, uint16_t));

bool telemetriaIsIdle();
void telemetriaSetTxInterval(unsigned long interval);

void taskTelemetria(void *parameter);

#endif
