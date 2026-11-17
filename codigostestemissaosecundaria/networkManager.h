#ifndef NETWORK_MANAGER_H
#define NETWORK_MANAGER_H

#include <WiFi.h>
#include <Arduino.h>

#define MAX_IMG_SIZE 30000

// Escrita pelo nucleo 0 (taskWiFi) e lida pelo nucleo 1 (taskTelemetria)
extern volatile bool imageReady;

extern const char* ssid;
extern const char* password;
extern const char*serverIP;
extern const uint16_t port;

// --- Declarações Externas ---
extern uint32_t currentTotalIndex;
extern float currentMissionTime;
extern size_t currentImgSize;
extern bool firstConnectionEver;

extern size_t currentImgSize;
extern uint32_t currentTotalIndex;
extern float currentMissionTime;

void setupNetwork();
void resetRadio();
void monitorConnection();
bool receiveImage();

// Missao secundaria: baixa a foto com o indice EXATO informado pela CAM (photo_index)
bool receiveImageIndex(int idx);

// Buffer com a última imagem recebida via WiFi (preenchido em receiveImage())
uint8_t* getImageBuffer();
uint32_t getImageSize();

// Resultado da consulta ao Orange para a deteccao em andamento
enum Veredito : uint8_t { V_PENDENTE, V_CLANDESTINA, V_AUTORIZADA, V_INDETERMINADO };

// Chamada pela telemetria quando o veredito chega (ignora se nao for a deteccao atual)
void m2Veredito(int photoIndex, Veredito v);

void taskWiFi(void *pvParameters);

#endif
