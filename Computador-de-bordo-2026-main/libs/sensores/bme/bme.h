#ifndef BME_H
#define BME_H

#include <Arduino.h>

//Inicializa o hardware, valida o ID (BME280/BMP280) e carrega as calibrações
bool bme_init();

//Lê os dados brutos via I2C e processa a compensação da Bosch
void bme_update();

//Retorna a temperatura processada em Graus Celsius (°C)
float bme_get_temperature();

//Retorna a pressão atmosférica processada em Hectopascals (hPa)
float bme_get_pressure();

//Retorna a umidade relativa do ar em porcentagem (%)
float bme_get_humidity();

//Calcula e retorna a altitude em Metros (m) baseada na pressão do nível do mar (ex: 1013.25)
float bme_get_altitude(float sea_level_hPa);

#endif //BME_H