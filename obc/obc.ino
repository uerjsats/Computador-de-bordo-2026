#include "sensores.h"
#include "telemetria.h"
#include <Wire.h>
#include <Arduino.h>

QueueHandle_t filaTelemetria;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  filaTelemetria = xQueueCreate(5, sizeof(sensorsData));

  //Cria task de sensores
  xTaskCreatePinnedToCore(
      taskSensores,      //função
      "TaskSensores",    //nome
      4096,              //stack
      NULL,              //parâmetros
      4,                 //prioridade
      NULL,              //handle
      1                  //core
  );

  xTaskCreatePinnedToCore(
      taskTelemetria,
      "TaskTelemetria",
      4096,
      NULL,
      3,
      NULL,
      1
  );

}

void loop() {
  
}
