#include "telemetria.h"

#define RF_FREQUENCY 920.5
#define TX_OUTPUT_POWER 20
#define LORA_BANDWIDTH 125.0
#define LORA_SPREADING_FACTOR 7
#define LORA_CODINGRATE 5

extern QueueHandle_t filaTelemetria;

static SX1262 radio = new Module(8, 14, 12, 13);

static volatile bool receivedFlag = false;

static uint8_t myAddress;
static uint8_t destAddress;

static bool idle = true;
static unsigned long txInterval = 300;
static unsigned long lastTxTime = 0;

static uint8_t rxBuffer[BUFFER_SIZE];
static uint8_t txBuffer[BUFFER_SIZE];

static void (*packetCallback)(uint8_t*, uint16_t) = nullptr;

static void setFlag()
{
    receivedFlag = true;
}

static void handleReceived(uint8_t* payload, uint16_t size)
{
    if (!validateHeader(payload, size)) {
        return;
    }

    uint8_t src = packetSrc(payload);
    uint8_t dst = packetDst(payload);

    // So aceita pacotes enderecados a este dispositivo e vindos do par esperado
    if (dst != myAddress || src != destAddress) {
        return;
    }

    // Repassa o pacote INTEIRO (com cabecalho) para o callback.
    // O callback usa packetType()/packetPayload()/packetPayloadSize() de protocol.h
    // para interpretar.
    if (packetCallback)
        packetCallback(payload, size);
}

void telemetriaInit(uint8_t myAddr, uint8_t destAddr)
{
    myAddress = myAddr;
    destAddress = destAddr;

    int state = radio.begin(RF_FREQUENCY, LORA_BANDWIDTH, LORA_SPREADING_FACTOR, LORA_CODINGRATE);
    if (state != RADIOLIB_ERR_NONE) {
        return;
    }

    radio.setOutputPower(TX_OUTPUT_POWER);
    radio.setPacketReceivedAction(setFlag);
    radio.startReceive();
}

void telemetriaProcess()
{
    if (receivedFlag) {
        receivedFlag = false;

        if (!idle) {
            radio.finishTransmit();
            idle = true;
            radio.startReceive();
            return;
        }

        int len = radio.getPacketLength();

        if (len > 0 && len <= BUFFER_SIZE) {
            int state = radio.readData(rxBuffer, len);
            if (state == RADIOLIB_ERR_NONE) {
                handleReceived(rxBuffer, len);
            }
        }

        radio.startReceive();
    }
}

void telemetriaSendPacket(uint8_t* data, uint16_t size, uint8_t type)
{
    if (!idle) return;
    if (millis() - lastTxTime < txInterval) return;
    if ((uint32_t)size + HEADER_SIZE > BUFFER_SIZE) return;

    uint8_t idx = buildHeader(txBuffer, type, myAddress, destAddress);
    memcpy(&txBuffer[idx], data, size);

    idle = false;
    radio.startTransmit(txBuffer, size + idx);
    lastTxTime = millis();
}

void telemetriaSendPacket(const char* payload, uint8_t type)
{
    telemetriaSendPacket((uint8_t*)payload, strlen(payload), type);
}

void telemetriaOnPacketReceived(void (*callback)(uint8_t*, uint16_t))
{
    packetCallback = callback;
}

bool telemetriaIsIdle()
{
    return idle;
}

void telemetriaSetTxInterval(unsigned long interval)
{
    txInterval = interval;
}

static void callbackTelemetria(uint8_t* data, uint16_t size)
{
    uint8_t type = packetType(data);
    const uint8_t* payload = packetPayload(data);
    uint16_t payloadLen = packetPayloadSize(size);

    if (type != TYPE_COMMAND) {
        return;
    }

    String cmd = "";

    for (uint16_t i = 0; i < payloadLen; i++) {
        cmd += (char)payload[i];
    }

    if (cmd == "0" || cmd == "5" || cmd == "6" ||
        cmd == "7" || cmd == "8" || cmd == "10" ||
        cmd == "11" || cmd == "13") {

        String msgMecanismo = String(ADDR_OBC) + ":" + cmd;
    }
}

void taskTelemetria(void *parameter)
{
    sensorsData dados;

    static unsigned long lastReqSuprimento = 0;
    static unsigned long reqSuprimentoInterval = 2000;

    telemetriaOnPacketReceived(callbackTelemetria);
    telemetriaInit(ADDR_OBC, ADDR_GROUND);

    while (true)
    {
        telemetriaProcess();

        if (xQueueReceive(filaTelemetria, &dados, 0) == pdPASS)
        {
            if (telemetriaIsIdle())
            {
                telemetriaSendPacket(
                    (uint8_t*)&dados,
                    sizeof(dados),
                    TYPE_SENSOR
                );
            }
        }
        //Serial.println("ENviou");

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}