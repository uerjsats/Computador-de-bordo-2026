#include "sensores.h"
#include <Wire.h>
#include <Arduino.h>


struct __attribute__((packed)) TelemetryFrame {
  float temperatura, umidade;
  float gyroX, gyroY, gyroZ;
  float accelX, accelY, accelZ;
  float pressao, altitude;
  float latitude, longitude;
  int32_t sats;
};

bool bmeOk = false;

void setup() {
  Serial.begin(115200);
  dhtBegin();
  Wire.begin();
  gpsInit();

  if (!mpuInit(false)) {
    Serial.println("ERRO: MPU NAO ENCONTRADO");
  }

  bmeOk = bme_init();
  if (!bmeOk) {
    Serial.println("ERRO: BME280 NAO ENCONTRADO");
  }
}

void loop() {
  float temperatura = NAN, umidade = NAN;
  bool dhtOk = dhtUpdate(temperatura, umidade);

  float gyroX, gyroY, gyroZ, accelX, accelY, accelZ;
  mpuUpdate(gyroX, gyroY, gyroZ, accelX, accelY, accelZ);

  float latitude, longitude;
  int sats;
  gpsUpdate(latitude, longitude, sats);


  Serial.println("DEBUG DHT");
  Serial.println("================");
  if (dhtOk) {
    Serial.print("Temperatura: ");
    Serial.println(temperatura);
    Serial.print("Umidade: ");
    Serial.println(umidade);
  } else {
    Serial.println("ERRO: falha na leitura do DHT");
  }

  Serial.println("DEBUG MPU");
  Serial.println("================");
  Serial.print("GyroX: ");
  Serial.println(gyroX);
  Serial.print("GyroY: ");
  Serial.println(gyroY);
  Serial.print("GyroZ: ");
  Serial.println(gyroZ);
  Serial.print("AccelX: ");
  Serial.println(accelX);
  Serial.print("AccelY: ");
  Serial.println(accelY);
  Serial.print("AccelZ: ");
  Serial.println(accelZ);

  Serial.println("DEBUG BME");
  Serial.println("================");
  if (bmeOk) {
    bme_update();
    Serial.print("Pressao (Pa): ");
    Serial.println(bme_get_pressure());
    Serial.print("Altitude (m): ");
    Serial.println(bme_get_altitude(101325.0f));
  } else {
    Serial.println("ERRO: BME280 nao inicializado");
  }

  Serial.println("DEBUG GPS");
  Serial.println("================");
  if (sats > 0) {
    Serial.print("Latitude: ");
    Serial.println(latitude, 6);
    Serial.print("Longitude: ");
    Serial.println(longitude, 6);
    Serial.print("Satelites: ");
    Serial.println(sats);
  } else {
    Serial.println("Aguardando fix de GPS (0 satelites)...");
  }


    // ---- EEPROM: grava o frame atual e le de volta para conferir ----
  TelemetryFrame frame = {
    temperatura, umidade,
    gyroX, gyroY, gyroZ,
    accelX, accelY, accelZ,
    bmeOk ? bme_get_pressure() : NAN,
    bmeOk ? bme_get_altitude(101325.0f) : NAN,
    latitude, longitude,
    (int32_t)sats
  };

  Serial.println("DEBUG EEPROM");
  Serial.println("================");
  Serial.print("Gravando frame no endereco: ");
  Serial.println(currentAddress);

  eepromWrite(currentAddress, frame);

  TelemetryFrame readBack;
  eepromReadBytes(currentAddress, (uint8_t*)&readBack, sizeof(readBack));

  Serial.print("Leitura de volta -> Temp: ");
  Serial.print(readBack.temperatura);
  Serial.print(" | Pressao: ");
  Serial.print(readBack.pressao);
  Serial.print(" | Sats: ");
  Serial.println(readBack.sats);

  currentAddress += sizeof(TelemetryFrame);
  if (currentAddress + (int)sizeof(TelemetryFrame) > EEPROM_MAX_SIZE) {
    currentAddress = 10; // volta ao inicio, preservando os 10 primeiros bytes reservados
    Serial.println("AVISO: EEPROM cheia, reiniciando endereco");
  }

  delay(5000);
}
