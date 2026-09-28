#ifndef LORARX_H
#define LORARX_H

#include <Arduino.h>
#include <RadioLib.h>
#include <stdint.h>
#include <string.h>

#define START_BYTE   0x7E

#define TYPE_SENSOR  0x01
#define TYPE_GPS     0x02
#define TYPE_GYRO    0x03
#define TYPE_IMAGE   0x10
#define TYPE_DEBUG   0x20
#define TYPE_COMMAND 0x30

#define ADDR_GROUND  0x01
#define ADDR_OBC     0x02

#define HEADER_SIZE  4
#define BUFFER_SIZE  256
#define PACKET_SIZE  180

#define MAX_IMAGE_SIZE       45900
#define IMAGE_CHUNK_TIMEOUT  5000

#pragma pack(push, 1)
struct sensorsData {
    uint32_t seconds;
    int16_t  temperatura;
    int16_t  umidade;
    int16_t  altitude;
    uint32_t pressao;
    int32_t  latitude;
    int32_t  longitude;
    uint8_t  sats;
    int16_t  roll;
    int16_t  pitch;
    int16_t  yaw;
};
#pragma pack(pop)

struct LoraRxStats {
    uint32_t totalRecebidos    = 0;
    uint32_t sensorCount       = 0;
    uint32_t imageCount        = 0;
    uint32_t commandCount      = 0;
    uint32_t desconhecidos     = 0;
    uint32_t comandosEnviados  = 0;
    uint32_t imagensCompletas  = 0;
    uint32_t imagensDescartadas = 0;
    float    ultimoRSSI = 0;
    float    ultimoSNR  = 0;
};

extern LoraRxStats loraStats;

inline uint8_t buildHeader(uint8_t* buf, uint8_t type, uint8_t src, uint8_t dst) {
    buf[0] = START_BYTE;
    buf[1] = type;
    buf[2] = src;
    buf[3] = dst;
    return HEADER_SIZE;
}

inline bool validateHeader(const uint8_t* buf, uint16_t size) {
    if (size < HEADER_SIZE) return false;
    if (buf[0] != START_BYTE) return false;
    return true;
}

inline uint8_t packetType(const uint8_t* buf) {
    return buf[1];
}

inline uint8_t packetSrc(const uint8_t* buf) {
    return buf[2];
}

inline uint8_t packetDst(const uint8_t* buf) {
    return buf[3];
}

inline const uint8_t* packetPayload(const uint8_t* buf) {
    return &buf[HEADER_SIZE];
}

inline uint16_t packetPayloadSize(uint16_t totalSize) {
    return (totalSize > HEADER_SIZE) ? (totalSize - HEADER_SIZE) : 0;
}

inline bool parseSensorData(const uint8_t* payload, uint16_t payloadSize, sensorsData* out) {
    if (payloadSize < sizeof(sensorsData)) return false;
    memcpy(out, payload, sizeof(sensorsData));
    return true;
}

void  lorarxInit(uint8_t myAddress, uint8_t destAddress);
void  lorarxProcess();
void  lorarxSendCommand(const char* comando);
bool  lorarxIsIdle();
void  lorarxSetTxInterval(unsigned long interval);
float lorarxGetRSSI();
float lorarxGetSNR();
void  lorarxPrintStats();

#endif
