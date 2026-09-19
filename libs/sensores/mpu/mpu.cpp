#include "mpu.h"
#include <Wire.h>
#include <Arduino.h>

//Endereços possíveis de um MPU
byte enderecos[] = {0x68, 0x69, 0x70, 0x71}, mpu = 0x00;

bool mpuInit(bool wire)
{
  //Se chamar a função com parâmetro de verdadeiro ela inicializa o i2c junto com o MPU
  if(wire){
    Wire.begin();
  }

  byte addr, id;

  for(byte i = 0; i < sizeof(enderecos); i++)
  {
    addr = enderecos[i];
    
    //Verificar os endereços
    Wire.beginTransmission(addr);
    Wire.write(0x75);

    if(Wire.endTransmission() == 0)
    {

      Wire.beginTransmission(addr);
      Wire.write(0x75);
      if(Wire.endTransmission(false) == 0)
      {
        Wire.requestFrom((int)addr, 1);
        if(Wire.available())
        {
          id = Wire.read();
          if(id == 0x68 || id == 0x70 || id == 0x71 || id == 0x73)
          {
            mpu = addr; //Endereço Real
            Serial.print("MPU Detectado com sucesso, seu endereço eh: ");
            Serial.print(mpu, HEX);
            Serial.print(" com WHO_AM_I: 0x");
            Serial.println(id, HEX);

            // Acorda o sensor encontrado
            Wire.beginTransmission(mpu);
            Wire.write(0x6B); // Registro PWR_MGMT_1
            Wire.write(0);    // Zera o modo sleep
            Wire.endTransmission(true);
            
            return true; // Sucesso na inicialização

          }
        }
      }
    
    }

  }

  return false;

}

void mpuUpdate(float &gyroX, float &gyroY, float &gyroZ, float &accelX, float &accelY, float &accelZ)
{
    long somaGX = 0;
    long somaGY = 0;
    long somaGZ = 0;

    long somaAX = 0;
    long somaAY = 0;
    long somaAZ = 0;

    int leiturasValidas = 0;

    for (int i = 0; i < 10; i++)
    {
        //Acelerômetro
        Wire.beginTransmission(mpu);
        Wire.write(0x3B); // ACCEL_XOUT_H

        if (Wire.endTransmission(false) != 0)
            continue;

        if (Wire.requestFrom((int)mpu, 6, true) != 6)
            continue;

        int16_t rawAX = (Wire.read() << 8) | Wire.read();
        int16_t rawAY = (Wire.read() << 8) | Wire.read();
        int16_t rawAZ = (Wire.read() << 8) | Wire.read();

        //Giroscópio
        Wire.beginTransmission(mpu);
        Wire.write(0x43); // GYRO_XOUT_H

        if (Wire.endTransmission(false) != 0)
            continue;

        if (Wire.requestFrom((int)mpu, 6, true) != 6)
            continue;

        int16_t rawGX = (Wire.read() << 8) | Wire.read();
        int16_t rawGY = (Wire.read() << 8) | Wire.read();
        int16_t rawGZ = (Wire.read() << 8) | Wire.read();


        //Soma valores brutos
        somaAX += rawAX;
        somaAY += rawAY;
        somaAZ += rawAZ;

        somaGX += rawGX;
        somaGY += rawGY;
        somaGZ += rawGZ;

        leiturasValidas++;
    }


    if (leiturasValidas > 0)
    {
        // MPU6050 para calibração:
        // Acelerômetro ±2g = 16384 LSB/g
        accelX = (somaAX / (float)leiturasValidas) / 16384.0;
        accelY = (somaAY / (float)leiturasValidas) / 16384.0;
        accelZ = (somaAZ / (float)leiturasValidas) / 16384.0;

        // Giroscópio ±250 °/s = 131 LSB/(°/s)
        gyroX = (somaGX / (float)leiturasValidas) / 131.0;
        gyroY = (somaGY / (float)leiturasValidas) / 131.0;
        gyroZ = (somaGZ / (float)leiturasValidas) / 131.0;
    }
}