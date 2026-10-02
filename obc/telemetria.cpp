#include "telemetria.h"
#include "networkManager.h"

#define RF_FREQUENCY 920.5
#define TX_OUTPUT_POWER 20
#define LORA_BANDWIDTH 125.0
#define LORA_SPREADING_FACTOR 7
#define LORA_CODINGRATE 5

static bool ignorarSup = false;           // true = descartando suprimento
static bool ultimaRespostaControle = false;
static unsigned long ignorarSupDesde = 0; // quando comecou a ignorar suprimento
#define TIMEOUT_CONTROLE_MS 5000          // seguranca: se controle nao responder, volta a ouvir suprimento

// Acumuladores de linha parcial de cada serial (usados por lerLinha)
static String accC;   // controle
static String accS;   // suprimento

// Linha aguardando envio via LoRa (movida da task para o escopo do arquivo
// para o callback poder descartar linha de suprimento obsoleta)
static String pendente;
static bool pendenteEhControle = false;

HardwareSerial controleSerial(2);
HardwareSerial supSerial(1);

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
    cmd.trim();

    int num = cmd.toInt();

    if (num >= 1 && num <= 11) {
        ignorarSup = true;          // ignora suprimento até a resposta de controle sair
        ignorarSupDesde = millis();

        // descarta qualquer coisa de suprimento ja recebida/acumulada
        while (supSerial.available()) supSerial.read();
        accS = "";
        if (!pendenteEhControle) pendente = "";   // linha de suprimento ainda nao enviada fica obsoleta

        controleSerial.println(cmd);
    }
    else if (num >= 12 && num <= 14) {
        ignorarSup = false;         // quer resposta do suprimento
        supSerial.println(cmd);
    }

    Serial.print("Comando Recebido de OBC: ");
    Serial.println(cmd);
}


void serialInit()
{

    controleSerial.begin(9600, SERIAL_8N1, 33, 34);
    supSerial.begin(9600, SERIAL_8N1, 19, 20);

}

bool telemetriaSendString(const char* texto)
{
    if (texto == nullptr) return false;

    uint16_t size = strlen(texto);

    if (size == 0) return false;
    if (!idle) return false;
    if (millis() - lastTxTime < txInterval) return false;
    if ((uint32_t)size + HEADER_SIZE > BUFFER_SIZE) return false;

    uint8_t idx = buildHeader(txBuffer, TYPE_STRING, myAddress, destAddress);
    memcpy(&txBuffer[idx], texto, size);

    idle = false;

    int state = radio.startTransmit(txBuffer, size + idx);

    if (state != RADIOLIB_ERR_NONE) {
        idle = true;
        return false;
    }

    lastTxTime = millis();
    return true;
}

bool telemetriaSendString(const String& texto)
{
    return telemetriaSendString(texto.c_str());
}

static String lerLinha(HardwareSerial& s, String& acc)
{
    while (s.available()) {
        char c = s.read();
        if (c == '\n') {
            String l = acc;
            acc = "";
            l.trim();
            return l;
        }
        if (c != '\r') acc += c;
    }
    return "";
}

String processarResposta()
{
    String r = lerLinha(controleSerial, accC);
    if (r.length()) {
        ultimaRespostaControle = true;
        return r;
    }

    ultimaRespostaControle = false;

    if (ignorarSup) {
        while (supSerial.available()) supSerial.read();
        accS = "";
        return "";
    }

    return lerLinha(supSerial, accS);
}

void taskTelemetria(void *parameter)
{
    sensorsData dados;

    telemetriaOnPacketReceived(callbackTelemetria);
    telemetriaInit(ADDR_OBC, ADDR_GROUND);

    unsigned long lastSensorSend = 0;

    serialInit();

    while (true)
    {
        telemetriaProcess();
        
        unsigned long now = millis();

        // timeout de seguranca: controle nao respondeu, volta a ouvir suprimento
        if (ignorarSup && (now - ignorarSupDesde > TIMEOUT_CONTROLE_MS)) {
            ignorarSup = false;
        }

        if (pendente.length() == 0) {
            pendente = processarResposta();
            if (pendente.length()) {
                Serial.println(pendente);
            }
            pendenteEhControle = ultimaRespostaControle;
        }

        if (pendente.length() && telemetriaSendString(pendente)) {
            if (pendenteEhControle) {
                ignorarSup = false;   // resposta de controle enviada: volta a ouvir suprimento
            }
            pendente = "";
            pendenteEhControle = false;
        }
        if (xQueueReceive(filaTelemetria, &dados, 0) == pdPASS)
        {
            if (telemetriaIsIdle() && now - lastSensorSend >= 2000)
            {
                telemetriaSendPacket(
                    (uint8_t*)&dados,
                    sizeof(dados),
                    TYPE_SENSOR
                );
                lastSensorSend = now;
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