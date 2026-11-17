#ifndef NETWORK_MANAGER_H
#define NETWORK_MANAGER_H

#include <WiFi.h>
#include <Arduino.h>

#define MAX_IMG_SIZE 30000

// Escrita pelo nucleo 0 (taskWiFi) e lida pelo nucleo 1 (taskTelemetria)
extern volatile bool imageReady;

extern const char* ssid;
extern const char* password;
extern const char* serverIP;
extern const uint16_t port;

// --- Declarações Externas ---
extern uint32_t currentTotalIndex;
extern float currentMissionTime;
extern size_t currentImgSize;
extern bool firstConnectionEver;

// Veredito da missao secundaria (Orange Pi -> OBC).
// Precisa vir ANTES de qualquer funcao que use o tipo Veredito.
enum Veredito { V_AUTORIZADA, V_CLANDESTINA, V_INDETERMINADO };

void setupNetwork();
bool receiveImage();
bool receiveImageIndex(int idx);
void m2Veredito(int photoIndex, Veredito v);
uint8_t* getImageBuffer();
uint32_t getImageSize();
void taskWiFi(void *parameter);

#endif