#include "lorarx.h"

void setup()
{
    Serial.begin(115200);
    lorarxInit(ADDR_GROUND, ADDR_OBC);
}

void loop()
{
    lorarxProcess();

    if (Serial.available()) {
        String comando = Serial.readStringUntil('\n');
        comando.trim();

        if (comando.length() > 0) {
            if (comando == "stats") {
                lorarxPrintStats();
            } else if (!lorarxIsIdle()) {
                Serial.println("Radio ocupado.");
            } else {
                // Split "15 600" (or "15,600") into command number and argument
                int sep = comando.indexOf(' ');
                if (sep < 0) sep = comando.indexOf(',');

                String first = (sep < 0) ? comando : comando.substring(0, sep);
                String rest  = (sep < 0) ? ""      : comando.substring(sep + 1);
                first.trim();
                rest.trim();

                int intCommand = first.toInt();

                if (intCommand >= 15 && intCommand <= 17) {
                    Serial.println("Comando de Missão Reconhecido");

                    if (intCommand == 15) {
                        long tempoMs = rest.toInt();
                        if (tempoMs <= 0) {
                            Serial.println("Uso: 15 <tempo_ms>  (ex: 15 600)");
                            return;
                        }
                        comando = "OP,START," + String(tempoMs);
                    } else if (intCommand == 16) {
                        comando = "OP,STOP";
                    } else {
                        comando = "OP,QRY";
                    }
                }

                lorarxSendCommand(comando.c_str());
                Serial.print("Comando enviado para OBC: ");
                Serial.println(comando);
            }
        }
    }
}