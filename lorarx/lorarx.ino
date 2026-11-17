#include "lorarx.h"

// O .py envia o comando SEM '\n'. O fim do comando e detectado por:
//   - '\n' ou '\r' (Serial Monitor), ou
//   - FIM_COMANDO_MS sem chegar byte novo (programa Python).
#define FIM_COMANDO_MS   50UL
#define TIMEOUT_ENVIO_MS 3000UL

static String usbAcc;                 // comando sendo montado
static unsigned long ultimoByteUSB = 0;
static String pendente;               // comando esperando o radio liberar
static unsigned long pendenteDesde = 0;

static String traduzir(const String& c)
{
    if (c == "15") return "OP,START,600";
    if (c == "16") return "OP,STOP";
    if (c == "17") return "OP,PING";
    return c;
}

static void tratarComando(String c)
{
    c.trim();
    if (c.length() == 0) return;
    if (c == "stats") { lorarxPrintStats(); return; }
    if (pendente.length()) {
        Serial.println("Aguarde: ainda enviando o comando anterior.");
        return;
    }
    pendente = traduzir(c);
    pendenteDesde = millis();
}

static void lerUSB()
{
    while (Serial.available()) {
        char ch = (char)Serial.read();
        ultimoByteUSB = millis();
        if (ch == '\n' || ch == '\r') {
            if (usbAcc.length()) { tratarComando(usbAcc); usbAcc = ""; }
        } else if (usbAcc.length() < 100) {
            usbAcc += ch;
        }
    }
    // Sem terminador: fecha o comando apos um tempo sem bytes novos
    if (usbAcc.length() && millis() - ultimoByteUSB >= FIM_COMANDO_MS) {
        tratarComando(usbAcc);
        usbAcc = "";
    }
}

static void enviarPendente()
{
    if (!pendente.length()) return;
    if (lorarxSendCommand(pendente.c_str())) {
        Serial.print("Comando enviado para OBC: ");
        Serial.println(pendente);
        pendente = "";
    } else if (millis() - pendenteDesde > TIMEOUT_ENVIO_MS) {
        Serial.print("ERRO: radio ocupado, comando descartado: ");
        Serial.println(pendente);
        pendente = "";
    }
}

void setup()
{
    Serial.begin(115200);
    lorarxInit(ADDR_GROUND, ADDR_OBC);
}

void loop()
{
    lorarxProcess();
    lerUSB();
    enviarPendente();
}