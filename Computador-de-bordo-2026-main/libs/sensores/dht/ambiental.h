#ifndef AMBIENTAL_H
#define AMBIENTAL_H

#include <Arduino.h>

void dhtBegin();
void dhtUpdate(float &temperatura, float &umidade);

#endif