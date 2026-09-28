#include "telemetria.h"
#include "networkManager.h"

#define RF_FREQUENCY 920.5
#define TX_OUTPUT_POWER 20
#define LORA_BANDWIDTH 125.0
#define LORA_SPREADING_FACTOR 7
#define LORA_CODINGRATE 5

#define PACKET_SIZE     180
#define MAX_IMAGE_SIZE  50000

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
struct ImageTransaction {
    uint8_t* data  = nullptr;
    uint16_t size  = 0;
    uint16_t index = 0;
    uint16_t total = 0;
    bool ativo     = false;
};

static uint8_t binaryBuffer[MAX_IMAGE_SIZE];
static uint8_t rleBuffer[MAX_IMAGE_SIZE];
static ImageTransaction imgTx;

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

static void binarize(uint8_t* input, uint8_t* output, int size, uint8_t threshold)
{
    for (int i = 0; i < size; i++) {
        output[i] = (input[i] > threshold) ? 1 : 0;
    }
}

static int encodeRLE(uint8_t* input, int size, uint8_t* output)
{
    int outIndex = 0;
    uint8_t current = input[0];
    uint8_t count = 1;

    for (int i = 1; i < size; i++) {
        if (input[i] == current && count < 255) {
            count++;
        } else {
            output[outIndex++] = count;
            output[outIndex++] = current;
            current = input[i];
            count = 1;
        }
    }

    output[outIndex++] = count;
    output[outIndex++] = current;

    return outIndex;
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
            int txState = radio.finishTransmit();
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

void telemetriaSendImageRaw(uint8_t* data, uint32_t size)
{
    if (data == nullptr || size == 0) return;

    if (size > MAX_IMAGE_SIZE) {
        return;
    }

    telemetriaSendImage(data, size);
}

void telemetriaSendImage(uint8_t* data, uint16_t size)
{
    if (imgTx.ativo) return;

    imgTx.data = data;
    imgTx.size = size;
    imgTx.index = 0;
    imgTx.total = (size + PACKET_SIZE - 1) / PACKET_SIZE;
    imgTx.ativo = true;

}

void telemetriaSendImageChunk()
{
    if (!imgTx.ativo) return;
    if (!idle) return;
    if (millis() - lastTxTime < txInterval) return;

    if (imgTx.index >= imgTx.total) {
        imgTx.ativo = false;
        return;
    }

    int start = imgTx.index * PACKET_SIZE;
    int remaining = imgTx.size - start;
    int len = (remaining > PACKET_SIZE) ? PACKET_SIZE : remaining;

    uint8_t packet[PACKET_SIZE + 7];

    uint8_t idx = buildHeader(packet, TYPE_IMAGE, myAddress, destAddress);
    packet[idx++] = (uint8_t)imgTx.index;
    packet[idx++] = (uint8_t)imgTx.total;
    packet[idx++] = (uint8_t)len;

    memcpy(packet + idx, imgTx.data + start, len);

    idle = false;

    int state = radio.startTransmit(packet, len + idx);

    if (state != RADIOLIB_ERR_NONE) {
        imgTx.ativo = false;
        idle = true;
        return;
    }

    lastTxTime = millis();
    imgTx.index++;
}

bool telemetriaIsImageSending()
{
    return imgTx.ativo;
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

        if (imageReady && !telemetriaIsImageSending())
        {
            uint8_t* imgBuf = getImageBuffer();
            uint32_t imgSize = getImageSize();

            if (imgBuf != nullptr && imgSize > 0)
            {
                telemetriaSendImageRaw(imgBuf, imgSize);
            }

            imageReady = false;
        }

        if (telemetriaIsImageSending())
        {
            telemetriaSendImageChunk();

            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}