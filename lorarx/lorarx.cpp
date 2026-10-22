#include "lorarx.h"

#define RF_FREQUENCY          915.0
#define TX_OUTPUT_POWER       20
#define LORA_BANDWIDTH        125.0
#define LORA_SPREADING_FACTOR 7
#define LORA_CODINGRATE       5

static SX1262 radio = new Module(8, 14, 12, 13);

static volatile bool receivedFlag = false;

static uint8_t myAddress;
static uint8_t destAddress;

static bool idle = true;
static unsigned long txInterval = 300;
static unsigned long lastTxTime = 0;

static uint8_t rxBuffer[BUFFER_SIZE];
static uint8_t txBuffer[BUFFER_SIZE];

LoraRxStats loraStats;

static uint8_t  imageBuffer[MAX_IMAGE_SIZE];
static uint32_t imageWritePos        = 0;
static uint8_t  imageExpectedChunks  = 0;
static uint8_t  imageNextChunk       = 0;
static bool     imageInProgress      = false;
static uint32_t imageLastChunkMillis = 0;

static void setFlag()
{
    receivedFlag = true;
}

static void resetImagem()
{
    imageInProgress      = false;
    imageWritePos        = 0;
    imageExpectedChunks  = 0;
    imageNextChunk       = 0;
}

static void finalizarImagem()
{
    Serial.print("IMAGE_BEGIN\n");
    Serial.printf("SIZE:%lu\n", (unsigned long)imageWritePos);
    Serial.write(imageBuffer, imageWritePos);
    Serial.print("IMAGE_END\n");

    loraStats.imagensCompletas++;
    resetImagem();
}

static void handleImage(const uint8_t* pl, uint16_t plSize)
{
    if (plSize < 3) {
        return;
    }

    loraStats.imageCount++;

    uint8_t chunkIndex = pl[0];
    uint8_t totalChunk = pl[1];
    uint8_t dataLen    = pl[2];
    const uint8_t* chunkData = pl + 3;

    if (plSize < (uint16_t)(3 + dataLen)) {
        return;
    }

    if (chunkIndex == 0) {
        if (imageInProgress) {
            loraStats.imagensDescartadas++;
        }
        resetImagem();
        imageInProgress     = true;
        imageExpectedChunks = totalChunk;
        imageNextChunk      = 0;
    }

    if (!imageInProgress) {
        return;
    }

    if (chunkIndex != imageNextChunk || totalChunk != imageExpectedChunks) {
        loraStats.imagensDescartadas++;
        resetImagem();
        return;
    }

    if (imageWritePos + dataLen > MAX_IMAGE_SIZE) {
        loraStats.imagensDescartadas++;
        resetImagem();
        return;
    }

    memcpy(imageBuffer + imageWritePos, chunkData, dataLen);
    imageWritePos += dataLen;
    imageNextChunk++;
    imageLastChunkMillis = millis();

    if (imageNextChunk == totalChunk) {
        finalizarImagem();
    }
}

static void handleSensor(const uint8_t* pl, uint16_t plSize, uint16_t totalSize)
{
    sensorsData dados;

    if (!parseSensorData(pl, plSize, &dados)) {
        return;
    }

    loraStats.sensorCount++;

    float temperatura = dados.temperatura / 100.0f;
    float umidade     = dados.umidade / 100.0f;
    float altitude    = dados.altitude / 10.0f;
    uint32_t pressao  = dados.pressao;
    float latitude    = dados.latitude / 10000000.0f;
    float longitude   = dados.longitude / 10000000.0f;
    float roll_deg    = dados.roll / 100.0f;
    float pitch_deg   = dados.pitch / 100.0f;
    float yaw_deg     = dados.yaw / 100.0f;

    float tempBat1    = dados.temperaturaBateria1 / 100.0f;
    float tempBat2    = dados.temperaturaBateria2 / 100.0f;
    float tensao      = dados.tensao / 1000.0f;
    float corrente    = dados.corrente / 1000.0f;

    Serial.print(dados.seconds); Serial.print(":");
    Serial.print(temperatura, 2); Serial.print(":");
    Serial.print(umidade, 2); Serial.print(":");
    Serial.print(altitude, 1); Serial.print(":");
    Serial.print(pressao); Serial.print(":");
    Serial.print(latitude, 7); Serial.print(":");
    Serial.print(longitude, 7); Serial.print(":");
    Serial.print(dados.sats); Serial.print(":");
    Serial.print(roll_deg, 2); Serial.print(":");
    Serial.print(pitch_deg, 2); Serial.print(":");
    Serial.print(yaw_deg, 2); Serial.print(":");
    Serial.print(tempBat1, 2); Serial.print(":");
    Serial.print(tempBat2, 2); Serial.print(":");
    Serial.print(tensao, 3); Serial.print(":");
    Serial.print(corrente, 3); Serial.print(":");
    Serial.print(loraStats.totalRecebidos); Serial.print(":");
    Serial.print(loraStats.ultimoRSSI); Serial.print(":");
    Serial.println(totalSize);
}

static void handleCommand(const uint8_t* pl, uint16_t plSize)
{
    loraStats.commandCount++;

    Serial.print("COMANDO_RX:");
    for (uint16_t i = 0; i < plSize; i++) {
        if (pl[i] < 0x10) Serial.print("0");
        Serial.print(pl[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
}

// Tipos de mensagem que a Orange Pi (via OBC) envia como TYPE_STRING.
static bool ehMensagemOrange(const char* s)
{
    static const char* const tipos[] = {
        "AC,", "EVT,", "ACK,", "NAK,", "STATUS,", "QRES,", "VEREDITO,", "OPSTATE,"
    };
    for (size_t i = 0; i < sizeof(tipos) / sizeof(tipos[0]); i++) {
        if (strncmp(s, tipos[i], strlen(tipos[i])) == 0) return true;
    }
    return false;
}

static void handleString(const uint8_t* pl, uint16_t plSize)
{
    char buf[BUFFER_SIZE];

    uint16_t n = (plSize < sizeof(buf) - 1)
                 ? plSize
                 : sizeof(buf) - 1;

    memcpy(buf, pl, n);
    buf[n] = '\0';

    while (n > 0 && (buf[n - 1] == '\r' || buf[n - 1] == '\n')) {
        buf[--n] = '\0';
    }

    if (n == 0) {
        return;
    }

    loraStats.stringCount++;

    // Mensagens do Orange Pi (missão ADS-B / clandestinas).
    // Aceita o prefixo opcional "ORG," ou o tipo direto (AC, EVT, ACK, ...).
    // Um pacote pode trazer varias linhas (lote de AC separado por '\n'):
    // cada uma sai como uma linha "ORANGE:<mensagem>".
    char* orgIni = buf;
    if (strncmp(buf, "ORG,", 4) == 0) orgIni = buf + 4;

    if (orgIni != buf || ehMensagemOrange(orgIni)) {
        char* p = orgIni;
        while (p != nullptr && *p) {
            char* nl = strchr(p, '\n');
            if (nl) *nl = '\0';
            size_t l = strlen(p);
            while (l > 0 && p[l - 1] == '\r') p[--l] = '\0';
            if (l > 0) {
                Serial.print("ORANGE:");
                Serial.println(p);
                loraStats.orangeCount++;
            }
            p = nl ? nl + 1 : nullptr;
        }
        return;
    }

    char* linha = buf;

    // Prefixos opcionais de origem
    if (buf[0] == 'C' && buf[1] == ':') {
        Serial.println("Controle de Atitude");
        return;
    }

    if (buf[0] == 'S' && buf[1] == ':') {
        linha = buf + 2;
    }

    // Se nao comeca com numero, nao e um comando conhecido
    if (!isdigit((unsigned char)linha[0])) {
        Serial.print("STR_RAW:");
        Serial.println(linha);
        return;
    }

    // Separa "cmd" e "resto" (o resto pode nao existir)
    char* sep   = strchr(linha, ':');
    char* resto = (char*)"";

    if (sep != nullptr) {
        *sep  = '\0';
        resto = sep + 1;
    }

    int cmd = atoi(linha);
    if (cmd == 2) {
        // Missão
        Serial.println("Missão");
    }
    else if (cmd == 3) {
        // Controle de atitude
        Serial.print("Controle de Atitude: ");
        Serial.println(resto);
    }
    else {
        // Comando numerico desconhecido: mostra a linha original
        if (sep != nullptr) *sep = ':';
        Serial.print("STR_RAW:");
        Serial.println(linha);
    }
}

static void handleReceived(uint8_t* payload, uint16_t size)
{
    if (!validateHeader(payload, size)) {
        return;
    }

    uint8_t src = packetSrc(payload);
    uint8_t dst = packetDst(payload);

    if (dst != myAddress || src != destAddress) {
        return;
    }

    loraStats.totalRecebidos++;
    loraStats.ultimoRSSI = radio.getRSSI();
    loraStats.ultimoSNR  = radio.getSNR();

    uint8_t type = packetType(payload);
    const uint8_t* pl = packetPayload(payload);
    uint16_t plSize = packetPayloadSize(size);

    if (type == TYPE_SENSOR) {
        handleSensor(pl, plSize, size);
    } else if (type == TYPE_IMAGE) {
        handleImage(pl, plSize);
    } else if (type == TYPE_COMMAND) {
        handleCommand(pl, plSize);
    } else if (type == TYPE_STRING) {
        handleString(pl, plSize);
    } else {
        loraStats.desconhecidos++;
        Serial.print("TIPO_DESCONHECIDO:0x");
        Serial.println(type, HEX);
    }
}

void lorarxInit(uint8_t myAddr, uint8_t destAddr)
{
    myAddress = myAddr;
    destAddress = destAddr;

    int state = radio.begin(RF_FREQUENCY, LORA_BANDWIDTH, LORA_SPREADING_FACTOR, LORA_CODINGRATE);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.print("Falha ao iniciar radio: ");
        Serial.println(state);
        return;
    }

    radio.setOutputPower(TX_OUTPUT_POWER);
    radio.setPacketReceivedAction(setFlag);
    radio.startReceive();
}

void lorarxProcess()
{
    if (!receivedFlag) {
        if (imageInProgress && (millis() - imageLastChunkMillis > IMAGE_CHUNK_TIMEOUT)) {
            loraStats.imagensDescartadas++;
            resetImagem();
        }
        return;
    }

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

// Retorna true se o comando foi realmente transmitido. false = rádio ocupado
// ou intervalo minimo nao cumprido: o chamador pode tentar de novo depois.
bool lorarxSendCommand(const char* comando)
{
    if (comando == nullptr) return false;
    if (!idle) return false;
    if (millis() - lastTxTime < txInterval) return false;

    uint16_t size = strlen(comando);
    if (size == 0) return false;
    if ((uint32_t)size + HEADER_SIZE > BUFFER_SIZE) return false;

    uint8_t idx = buildHeader(txBuffer, TYPE_COMMAND, myAddress, destAddress);
    memcpy(&txBuffer[idx], comando, size);

    idle = false;
    int state = radio.startTransmit(txBuffer, size + idx);
    if (state != RADIOLIB_ERR_NONE) {
        idle = true;
        radio.startReceive();
        return false;
    }
    lastTxTime = millis();

    loraStats.comandosEnviados++;
    return true;
}

bool lorarxIsIdle()
{
    return idle;
}

void lorarxSetTxInterval(unsigned long interval)
{
    txInterval = interval;
}

float lorarxGetRSSI()
{
    return radio.getRSSI();
}

float lorarxGetSNR()
{
    return radio.getSNR();
}

void lorarxPrintStats()
{
    Serial.println();
    Serial.println("========== DEBUG STATS ==========");
    Serial.print("Total recebidos     : "); Serial.println(loraStats.totalRecebidos);
    Serial.print("  - Sensores        : "); Serial.println(loraStats.sensorCount);
    Serial.print("  - Imagem (chunks) : "); Serial.println(loraStats.imageCount);
    Serial.print("  - Comandos RX     : "); Serial.println(loraStats.commandCount);
    Serial.print("  - Strings         : "); Serial.println(loraStats.stringCount);
    Serial.print("    (msgs Orange)   : "); Serial.println(loraStats.orangeCount);
    Serial.print("  - Desconhecidos   : "); Serial.println(loraStats.desconhecidos);
    Serial.print("Comandos enviados   : "); Serial.println(loraStats.comandosEnviados);
    Serial.print("Imagens completas   : "); Serial.println(loraStats.imagensCompletas);
    Serial.print("Imagens descartadas : "); Serial.println(loraStats.imagensDescartadas);
    Serial.print("Ultimo RSSI         : "); Serial.print(loraStats.ultimoRSSI); Serial.println(" dBm");
    Serial.print("Ultimo SNR          : "); Serial.print(loraStats.ultimoSNR); Serial.println(" dB");
    Serial.println("==================================");
}