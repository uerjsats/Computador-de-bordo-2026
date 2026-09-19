#include <DHT.h>
#include "ambiental.h"

#define DHTPIN 48
#define DHTTYPE DHT22

DHT dht(DHTPIN, DHTTYPE);

void dhtBegin() {
  dht.begin();
}

void dhtUpdate(float &temperatura, float &umidade) {
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (!isnan(h) && !isnan(t)) {
    umidade = h;
    temperatura = t;
  }
}