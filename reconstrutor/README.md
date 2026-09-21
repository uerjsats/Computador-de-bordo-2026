# EstacaoSolo

Estação de solo da missão **UERJ SATS — Computador de Bordo (LASC 2026)**.
Recebe na base o stream serial da Heltec e o distribui para dois programas:
**AbaTrack** (telemetria/sensores/debug) e **`sat_image_receiver.py`** (imagem JPEG).

## O problema — 1 porta serial, 2 informações

A estação de solo entrega **um único stream serial misto** numa porta física,
com três conteúdos intercalados:

- **Telemetria** — 1 linha/s, 18 campos separados por `:` → **AbaTrack**.
- **Debug** — blocos de texto esporádicos.
- **Imagem JPEG** — `IMAGE_BEGIN / SIZE:<n> / <n bytes crus> / IMAGE_END` →
  **`sat_image_receiver.py`**.

No Windows, **1 porta COM = 1 processo** (vale até para portas virtuais com0com).
Para ter dois consumidores simultâneos usamos **fan-out**: um hub lê a porta real
UMA vez e duplica os bytes para dois pares de portas virtuais.

```
Placa (USB/LoRa, 1 COM real)
        │
        ▼
[heltec_leitor_central]  (abre a COM real UMA vez)
   ├── escreve COM12 ──(com0com)── COM13 ──►  AbaTrack            (telemetria)
   └── escreve COM14 ──(com0com)── COM15 ──►  sat_image_receiver  (imagem)
```

> Como o fan-out entrega o stream **completo** (inclusive os bytes do JPEG) para
> as duas pontas, o AbaTrack também recebe a imagem na COM13 — ele precisa
> **ignorá-la**. Isso motivou as 3 correções aplicadas (ver
> [`LEIA-ME_CORRECOES.md`](LEIA-ME_CORRECOES.md)).

## As 3 correções no AbaTrack

Bugs **reais de produção** — disparam sempre que uma rajada de bytes (imagem,
debug) compartilha a linha do AbaTrack. Detalhe completo em
[`LEIA-ME_CORRECOES.md`](LEIA-ME_CORRECOES.md).

1. **Portas com0com invisíveis** (`UI/UI2.py`) — COM13 não aparecia no menu;
   passa a mesclar o registro `SERIALCOMM`.
2. **Crash na 1ª imagem** (`integracao/adaptador_arduino.py`) — `decode("utf-8")`
   estourava nos bytes do JPEG; passa a usar `errors="ignore"`.
3. **Congela após minutos** (`UI/thread_main.py`) — `msleep(1000)` enchia o
   buffer com0com nas rajadas; reduzido para `msleep(10)`.

## Como rodar (resumo)

Pré-requisitos: **Python 3**, dependências (`pip install -r requirements_*.txt`)
e **com0com 2.2.2.0 assinada** (ver porquê da versão em `LEIA-ME_CORRECOES.md`).
Crie dois pares no *Setup Command Prompt* do com0com:

```
install PortName=COM12 PortName=COM13
install PortName=COM14 PortName=COM15
```

**Abra os dois leitores ANTES de iniciar a transmissão.**

- Abra o abatrack e conecte na porta COM13
- Abra o image receiver e selecione a porta COM14
- Conecte o microcontrolador referente a base
- Execute helte_leitor_central.exe

Passo a passo completo, solução de problemas e a discussão do com0com:
**[`LEIA-ME_CORRECOES.md`](LEIA-ME_CORRECOES.md)** e
**[`LEIA-ME_TESTE_WINDOWS.md`](LEIA-ME_TESTE_WINDOWS.md)**.
