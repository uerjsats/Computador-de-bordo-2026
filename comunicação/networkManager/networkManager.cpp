#include "NetworkManager.h"
#include "esp_wifi.h"
#include <Arduino.h>
#include <WiFi.h>

int nextIndexToRequest = 0;
uint32_t lastTotalIndex = 0xFFFFFFFF;

unsigned long lastProcessTime = 0;
const unsigned long PROCESS_DELAY = 60000;

uint8_t fb_buf[MAX_IMG_SIZE];

WiFiClient client;

bool conectedOnce = false;
bool firstConnectionEver = true;

size_t currentImgSize = 0;
uint32_t currentTotalIndex = 0;
float currentMissionTime = 0.0;

//Dados do Wifi da Missão
const char* ssid = "VANTsat_AP";
const char* password = "uerjsats123";
const char* serverIP = "192.168.4.1";
const uint16_t port = 8888;

void setupNetwork()
{
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    Serial.println("conectando em...");

    unsigned long startAttemptTime = millis();

    while (
        WiFi.status() != WL_CONNECTED &&
        millis() - startAttemptTime < 15000
    )
    {
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.println("Conectado com Sucesso");
    }
    else
    {
        Serial.println("falha de conexao");
    }
}

void resetRadio()
{
    Serial.println("resetando radio e buffers TCP");

    conectedOnce = false;

    client.stop();

    esp_wifi_stop();
    esp_wifi_deinit();

    vTaskDelay(pdMS_TO_TICKS(1500));

    wifi_init_config_t cfg =
        WIFI_INIT_CONFIG_DEFAULT();

    esp_wifi_init(&cfg);

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();

    WiFi.begin(ssid, password);

    Serial.println("Radio reiniciado");
}

void monitorConnection()
{
    //reconecta TCP
    if (
        WiFi.status() == WL_CONNECTED &&
        !client.connected()
    )
    {
        if (client.connect(serverIP, port))
        {
            client.setNoDelay(true);
            client.setTimeout(4000);

            //RESET DE MISSÃO (MESMO FLUXO)
            if (firstConnectionEver)
            {
                client.println("R");
                client.flush();

                firstConnectionEver = false;

                Serial.println("primeira conexao — resetando servidor");

                // espera sem travar watchdog
                unsigned long startWait =
                    millis();

                while (
                    millis() - startWait < 5000
                )
                {
                    vTaskDelay(
                        pdMS_TO_TICKS(50)
                    );
                }
            }

            conectedOnce = true;
        }
    }

    // queda de conexão
    if (
        conectedOnce &&
        (
            WiFi.status() != WL_CONNECTED ||
            !client.connected()
        )
    )
    {
        resetRadio();
    }
}

//Recebimento de missão
bool receiveImage()
{
    monitorConnection();

    if (!client.connected())
    {
        return false;
    }

    client.printf(
        "GET:%d\n",
        nextIndexToRequest
    );

    client.flush();

    unsigned long startMs = millis();

    while (!client.available())
    {
        if (
            millis() - startMs > 5000
        )
        {
            Serial.println("servidor nao respondeu (timeout)");

            client.stop();
            conectedOnce = false;

            return false;
        }

        // FIX WATCHDOG
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    String header =
        client.readStringUntil('\n');

    if (!header.startsWith("START:"))
    {
        Serial.println("header invalido recebido");

        client.stop();

        return false;
    }

    int partsFound = 0;
    String parts[6];

    int lastPos = 0;

    for (
        int i = 0;
        i < header.length() &&
        partsFound < 6;
        i++
    )
    {
        if (
            header[i] == ':' ||
            i == header.length() - 1
        )
        {
            parts[partsFound++] =
                header.substring(
                    lastPos,
                    (
                        i ==
                        header.length() - 1
                    )
                    ? i + 1
                    : i
                );

            lastPos = i + 1;
        }
    }

    if (partsFound < 6)
    {
        client.stop();
        return false;
    }

    String currentTipoFigura =
        parts[1];

    currentImgSize =
        (size_t)parts[3].toInt();

    currentTotalIndex =
        (uint32_t)parts[4].toInt();

    currentMissionTime =
        parts[5].toFloat();

    bool isNewImage =
        (
            currentTotalIndex >
            lastTotalIndex
        )
        ||
        (
            lastTotalIndex ==
            0xFFFFFFFF
        );

    if (
        lastTotalIndex !=
        0xFFFFFFFF &&
        lastTotalIndex >= 100
    )
    {
        if (
            currentTotalIndex <
            (
                lastTotalIndex - 100
            )
        )
        {
            isNewImage = true;
        }
    }

    if (!isNewImage)
    {
        uint8_t discard[512];

        size_t totalDiscarded = 0;

        unsigned long timeout =
            millis();

        while (
            totalDiscarded <
            currentImgSize &&
            millis() - timeout <
            5000
        )
        {
            if (client.available())
            {
                size_t toRead =
                    min(
                        (
                            size_t
                        )
                        client.available(),
                        sizeof(discard)
                    );

                size_t n =
                    client.read(
                        discard,
                        toRead
                    );

                totalDiscarded += n;

                timeout = millis();
            }

            // FIX WATCHDOG
            vTaskDelay(
                pdMS_TO_TICKS(1)
            );
        }

        client.readStringUntil('\n');

        return false;
    }

    if (currentImgSize > MAX_IMG_SIZE)
    {
        Serial.println("imagem maior que buffer, descartando");

        uint8_t discard[512];
        size_t totalDiscarded = 0;
        unsigned long timeout = millis();

        while (totalDiscarded < currentImgSize && millis() - timeout < 5000)
        {
            if (client.available())
            {
                size_t toRead = min((size_t)client.available(), sizeof(discard));
                size_t n = client.read(discard, toRead);
                totalDiscarded += n;
                timeout = millis();
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        client.readStringUntil('\n');
        return false;
    }

    const uint8_t
    SYNC_START[] =
    {
        0xAA,
        0xBB,
        0xCC,
        0xDD
    };

    const uint8_t
    SYNC_END[] =
    {
        0xEE,
        0xFF
    };

    String logPayload =
        "TIPO:" +
        currentTipoFigura +
        ", ID:" +
        String(nextIndexToRequest) +
        ", TOT:" +
        String(currentTotalIndex) +
        ", T:" +
        String(
            currentMissionTime,
            3
        );

    uint32_t logSize =
        logPayload.length();

    Serial.write(
        SYNC_START,
        4
    );

    Serial.write(0x01);

    Serial.write(
        (uint8_t*)&logSize,
        4
    );

    Serial.print(logPayload);

    Serial.write(
        SYNC_END,
        2
    );
    
    Serial.write(
        SYNC_START,
        4
    );

    Serial.write(0x02);

    uint32_t imgSize32 =
        (
            uint32_t
        )
        currentImgSize;

    Serial.write(
        (uint8_t*)&imgSize32,
        4
    );

    size_t totalRead = 0;

    uint8_t buffer[1024];

    unsigned long
    downloadStart =
        millis();

    while (
        totalRead <
        currentImgSize &&
        (
            millis() -
            downloadStart <
            10000
        )
    )
    {
        if (client.available())
        {
            size_t bytesToRead =
                min(
                    (
                        size_t
                    )
                    client.available(),
                    currentImgSize
                    -
                    totalRead
                );

            if (
                bytesToRead >
                sizeof(buffer)
            )
            {
                bytesToRead =
                    sizeof(buffer);
            }

            size_t n =
                client.read(
                    buffer,
                    bytesToRead
                );

            Serial.write(
                buffer,
                n
            );

            // Guarda também no buffer local, para uso pelo LoRa (taskTelemetria)
            memcpy(
                fb_buf + totalRead,
                buffer,
                n
            );

            totalRead += n;

            downloadStart =
                millis();
        }

        // FIX WATCHDOG
        vTaskDelay(
            pdMS_TO_TICKS(1)
        );
    }

    Serial.write(
        SYNC_END,
        2
    );
    
    if (
        totalRead ==
        currentImgSize
    )
    {
        client.readStringUntil(
            '\n'
        );

        lastTotalIndex =
            currentTotalIndex;

        nextIndexToRequest =
            (
                nextIndexToRequest
                + 1
            ) % 10;

        Serial.println("download de imagem concluido");

        return true;
    }

    Serial.println("download incompleto");

    client.stop();

    return false;
}

//Buffer de Image
uint8_t* getImageBuffer()
{
    return fb_buf;
}

uint32_t getImageSize()
{
    return (uint32_t)currentImgSize;
}