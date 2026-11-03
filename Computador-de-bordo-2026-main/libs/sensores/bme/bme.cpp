#include <Arduino.h>
#include "bme.h"
#include <Wire.h>
#include <math.h>

//Endereço I2C padrão do sensor
#define BME280_ADDR 0x76

//Registradores auxiliares usados na inicialização
#define BME280_REG_RESET  0xE0
#define BME280_REG_STATUS 0xF3

//Estrutura para armazenar as calibrações de fábrica da Bosch
struct BME280_Calib {
    uint16_t dig_T1; int16_t dig_T2, dig_T3;
    uint16_t dig_P1; int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
    uint8_t  dig_H1; int16_t dig_H2; uint8_t  dig_H3; int16_t dig_H4, dig_H5; int8_t   dig_H6;
};

//Variáveis Globais de Controle e Armazenamento
static BME280_Calib calib;
static int32_t t_fine = 0; //Fator de correção cruzada vital exigido pela Bosch
static float current_temperature = 0;  // °C
static float current_pressure = 0;     // Pa (Pascal)
static float current_humidity = 0;     // %RH

//Escreve um byte em um registrador específico do sensor
void bme_write_reg(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(BME280_ADDR);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

// Lê um bloco de bytes de forma segura tratando o tipo do endereço
void bme_read_regs(uint8_t reg, uint8_t* buffer, uint8_t len) {
    Wire.beginTransmission((uint8_t)BME280_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false); // Repeated Start para manter o barramento

    // Força o casting correto do endereço para o Wire do Arduino
    uint8_t bytesReceived = Wire.requestFrom((uint8_t)BME280_ADDR, len);

    for (uint8_t i = 0; i < len; i++) {
        if (i < bytesReceived && Wire.available()) {
            buffer[i] = Wire.read();
        } else {
            buffer[i] = 0x00; // Proteção caso o barramento falhe
        }
    }
}

// Lê um único registrador de 8 bits (usado para status/id)
uint8_t bme_read_reg8(uint8_t reg) {
    uint8_t v = 0;
    bme_read_regs(reg, &v, 1);
    return v;
}

void bme_compensate_temperature(int32_t adc_T) {
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)calib.dig_T1 << 1))) * ((int32_t)calib.dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)calib.dig_T1))) >> 12) * ((int32_t)calib.dig_T3)) >> 14;
    t_fine = var1 + var2;
    current_temperature = (float)((t_fine * 5 + 128) >> 8) / 100.0f;
}

void bme_compensate_pressure(int32_t adc_P) {
    int64_t var1 = ((int64_t)t_fine) - 128000;
    int64_t var2 = var1 * var1 * (int64_t)calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)calib.dig_P3) >> 8) + ((var1 * (int64_t)calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calib.dig_P1) >> 33;

    if (var1 == 0) { current_pressure = 0; return; } // só acontece se a calibração estiver corrompida/zerada

    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)calib.dig_P8) * p) >> 19;
    // Resultado em formato Q24.8 (Pa * 256) -> dividir por 256 dá Pascal (Pa)
    current_pressure = (float)(((p + var1 + var2) >> 8) + (((int64_t)calib.dig_P7) << 4)) / 256.0f;
}

void bme_compensate_humidity(int32_t adc_H) {
    int32_t v_x1_u32r = (t_fine - ((int32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)calib.dig_H4) << 20) - (((int32_t)calib.dig_H5) * v_x1_u32r)) +
                   ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)calib.dig_H6)) >> 10) *
                   (((v_x1_u32r * ((int32_t)calib.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
                   ((int32_t)calib.dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)calib.dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    current_humidity = (float)(v_x1_u32r >> 12) / 1024.0f;
}

//Inicializa o hardware, lê o ID e baixa as tabelas de calibração do chip
bool bme_init() {
    uint8_t id = 0;
    bme_read_regs(0xD0, &id, 1);
    if (id != 0x60 && id != 0x58) return false; //Valida se é BME280 (0x60) ou BMP280 (0x58)

    // Soft reset: garante um estado limpo do chip antes de ler a calibração
    bme_write_reg(BME280_REG_RESET, 0xB6);
    delay(5); // tempo de start-up após reset (datasheet: ~2ms), só ocorre 1x na inicialização
    uint32_t t0 = millis();
    while (bme_read_reg8(BME280_REG_STATUS) & 0x01) {
        if (millis() - t0 > 100) break; // timeout de segurança, evita travar o init
    }

    //Ler os coeficientes de Temperatura e Pressão (T1 a P9)
    uint8_t b1[24];
    bme_read_regs(0x88, b1, 24);
    calib.dig_T1 = (b1[1] << 8) | b1[0];   calib.dig_T2 = (b1[3] << 8) | b1[2];   calib.dig_T3 = (b1[5] << 8) | b1[4];
    calib.dig_P1 = (b1[7] << 8) | b1[6];   calib.dig_P2 = (b1[9] << 8) | b1[8];   calib.dig_P3 = (b1[11] << 8) | b1[10];
    calib.dig_P4 = (b1[13] << 8) | b1[12]; calib.dig_P5 = (b1[15] << 8) | b1[14]; calib.dig_P6 = (b1[17] << 8) | b1[16];
    calib.dig_P7 = (b1[19] << 8) | b1[18]; calib.dig_P8 = (b1[21] << 8) | b1[20]; calib.dig_P9 = (b1[23] << 8) | b1[22];

    //Ler os coeficientes de Umidade (H1 a H6)
    bme_read_regs(0xA1, &calib.dig_H1, 1);
    uint8_t b2[7];
    bme_read_regs(0xE1, b2, 7);
    calib.dig_H2 = (b2[1] << 8) | b2[0];
    calib.dig_H3 = b2[2];
    calib.dig_H4 = (b2[3] << 4) | (b2[4] & 0x0F);
    calib.dig_H5 = (b2[5] << 4) | (b2[4] >> 4);
    calib.dig_H6 = (int8_t)b2[6];
    // Aqui já sabemos com certeza a causa raiz em vez de "pressão zero" misterioso.
    if (calib.dig_P1 == 0) return false;

    //Configurar amostragem (Oversampling) de Umidade (Reg 0xF2) - deve ser escrito ANTES do 0xF4
    bme_write_reg(0xF2, 0x01); // Umidade x1

    //Configurar Controle Geral (Reg 0xF4): Temp x1, Pressão x1, Modo Normal
    bme_write_reg(0xF4, 0x27);
    return true;
}

//Executa a leitura sequencial em lote e atualiza as variáveis internas globais.
void bme_update() {
    uint8_t raw[8];
    //Coleta consecutiva de dados: Pressão (0xF7-0xF9), Temp (0xFA-0xFC), Umidade (0xFD-0xFE)
    bme_read_regs(0xF7, raw, 8);

    // Converte os bytes brutos do barramento para inteiros de 20 e 16 bits (ADC)
    int32_t adc_P = ((int32_t)raw[0] << 12) | ((int32_t)raw[1] << 4) | ((int32_t)raw[2] >> 4);
    int32_t adc_T = ((int32_t)raw[3] << 12) | ((int32_t)raw[4] << 4) | ((int32_t)raw[5] >> 4);
    int32_t adc_H = ((int32_t)raw[6] << 8)  | (int32_t)raw[7];

    // ORDEM CRÍTICA PARA FUNCIONAMENTO: A temperatura calcula o 't_fine' usado para ajustar Pressão e Umidade
    bme_compensate_temperature(adc_T);
    bme_compensate_pressure(adc_P);
    bme_compensate_humidity(adc_H);
}

// Retorna a temperatura estável calculada, em °C
float bme_get_temperature() { return current_temperature; }

// Retorna a pressão em Pascal (Pa) — NÃO em hPa
float bme_get_pressure() { return current_pressure; }

// Retorna a umidade relativa calculada, em %
float bme_get_humidity() { return current_humidity; }

float bme_get_altitude(float sea_level_Pa = 101325.0f) {
    return 44330.0f * (1.0f - powf(current_pressure / sea_level_Pa, 0.1903f));
}