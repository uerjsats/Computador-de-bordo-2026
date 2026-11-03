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
                int intCommand = comando.toInt();

                if (intCommand >= 15 && intCommand <= 17) { 
                    Serial.println("Comando de Missão Reconhecido"); 

                    if (intCommand == 15) { 
                        comando = "OP, START, 600";
                    } else if (intCommand == 16) { 
                        comando = "OP, STOP"; 
                    } else { 
                        comando = "OP, QRY"; 
                    }
                }

                lorarxSendCommand(comando.c_str());
                Serial.print("Comando enviado para OBC: ");
                Serial.println(comando);
            }
        }
    }
}
