#ifndef SENSORES_H
#define SENSORES_H

#include <Arduino.h>
#include <TinyGPSPlus.h>

//dht 
void dhtBegin();
void dhtUpdate(float &temperatura, float &umidade);

//mpu 
bool mpuInit(bool wire); 
void mpuUpdate(float &gyroX,  float &gyroY,  float &gyroZ,  float &accelX,  float &accelY,  float &accelZ);

//bme - Inicializa o hardware, valida o ID (BME280/BMP280) e carrega as calibrações
bool bme_init(); 

//Lê os dados brutos via I2C e processa a compensação da Bosch
void bme_update();

//Retorna a temperatura processada em Graus Celsius (°C)
float bme_get_temperature();

//Retorna a pressão atmosférica processada em Pascal (Pa)
float bme_get_pressure(); 

//Retorna a umidade relativa do ar em porcentagem (%)
float bme_get_humidity(); 

//Calcula e retorna a altitude em Metros (m) baseada na pressão do nível do mar (ex: 101325.0f)
float bme_get_altitude(float sea_level_Pa);

//GPS 
void gpsInit(); 
void gpsUpdate(float &latitude, float &longitude, int &sats); 

//EEPROM 
#define EEPROM_I2C_ADDRESS 0x50 
#define EEPROM_PAGE_SIZE 16
#define EEPROM_MAX_SIZE 1484 

extern int currentAddress; 

void eepromWriteByte(uint16_t address, uint8_t* data, uint16_t length);  
void eepromReadBytes(uint16_t address, uint8_t* data, uint16_t length);   

template<class T> 
void eepromWrite(uint16_t address, const T &value) { 
    eepromWriteByte(address, (uint8_t*)&value, sizeof(T)); 
}

void taskSensores(void *pvParameters);

#endif