# Sistema de Computador de Bordo

## Visão Geral

Este projeto consiste no computador de bordo desenvolvido pela equipe UERJSATS

O sistema é responsável por:

* Coletar dados de sensores embarcados.
* Receber imagens capturadas pela câmera do cubesat via rede Wi-Fi.
* Receber e transmitir telemetria através de rádio LoRa.
* Armazenar dados críticos em memória EEPROM.
* Encaminhar imagens e informações para a estação base.
* Gerenciar comunicação entre subsistemas da missão.
* Executar múltiplas tarefas simultaneamente utilizando FreeRTOS.

---

# Arquitetura Geral

```text
                ┌─────────────┐
                │  Payload    │
                │ (Câmera)    │
                └──────┬──────┘
                       │ WiFi
                       ▼
             ┌──────────────────┐
             │ Computador de    │
             │ Bordo (ESP32)    │
             └──────┬───────────┘
                    │
      ┌─────────────┼─────────────┐
      │             │             │
      ▼             ▼             ▼
   Sensores       LoRa         EEPROM
      │             │             │
      └──────┬──────┴──────┬──────┘
             ▼             ▼
        Estação Base   Armazenamento
```

---

# Principais Funcionalidades

## Aquisição de Telemetria

O sistema coleta continuamente:

### GPS

* Latitude
* Longitude
* Número de satélites

### Sensores Ambientais (BME280 e DHT)

* Temperatura
* Umidade
* Pressão atmosférica
* Altitude estimada

### Unidade Inercial (MPU)

* Velocidade angular / Giroscópio (Roll, Pitch, Yaw)

### Medições de Energia e Bateria (Slave EPS)

* Temperatura da Bateria 1
* Temperatura da Bateria 2
* Tensão do barramento
* Corrente consumida

---

## Comunicação LoRa

Responsável pela transmissão de:

* Dados de telemetria
* Estados do sistema
* Respostas de subsistemas
* Possível transmissão de imagens fragmentadas

Características:

* Comunicação de longa distância
* Baixo consumo energético
* Operação independente da rede Wi-Fi

---

## Recepção de Imagens

O módulo Wi-Fi:

* Conecta-se a câmera.
* Solicita novas imagens.
* Recebe os frames.
* Armazena os dados em buffer.
* Encaminha para a estação base via interface serial.

---

## Armazenamento em EEPROM

O sistema salva informações importantes da missão para:

* Recuperação pós-voo.
* Backup da telemetria.
* Análise posterior.

O armazenamento é realizado apenas quando critérios operacionais são atendidos.

---

## Comunicação entre Subsistemas

Existem canais de comunicação destinados a:

### Controle de Atitude

Recebe e envia comandos relacionados à estabilização e navegação.

### Sistema de Energia

Recebe e envia informações sobre alimentação elétrica e gerenciamento energético.

---

## Multitarefa com FreeRTOS

O sistema utiliza tarefas independentes para garantir:

* Coleta contínua dos sensores.
* Comunicação simultânea.
* Processamento de imagens.
* Armazenamento em memória.

---

# Estrutura do Projeto

Todo o código-fonte do Computador de Bordo (OBC) está contido no diretório `obc/`.

As bibliotecas separadas para serem reaproveitadas estão no diretório `libs/`.

Os scripts referente a reconstrução de imagens estão no diretório `reconstrutor/`.

## Arquivo Principal

### `obc/obc.ino`

Ponto de entrada do sistema.

Responsabilidades:

* Inicialização do ESP32.
* Inicialização do barramento I2C.
* Criação das filas de comunicação.
* Criação das tarefas FreeRTOS.
* Gerenciamento geral do sistema.

---

# Sensores

## `Sensores.h`

Declarações das funções referentes à:
* GPS;
* DHT22;
* MPU6050;
* BME280;
* EEPROM.

## `Sensores.cpp`

Responsável por:

* Criação da task que realiza a leitura de sensores;
* Inicialização de sensores;
* Leitura de sensores;
* Gerenciamento da escrita de dados na EEPROM

---

# Comunicação

## `telemetria.cpp`

Responsável por:

* Inicializar o rádio;
* Processamento de dados da base;
* Fragmentação de imagem em chunks;
* Envio de chunks para a base;
* Envio de dados para a base.

## `telemetria.h`
Responsável por:

* Declaração de Funções;
* Armazenar protocolo compartilhado entre a base e o satélite.

## `NetworkManager.cpp`

Responsável por:

* Conexão com o ponto de acesso do drone.
* Monitoramento da conexão.
* Recepção de imagens.
* Gerenciamento de buffers.
* Controle da missão de captura.

## `NetworkManager.h`

Responsável por:

* Declaração de Funções;
* Armazenar protocolo compartilhado entre a base e o satélite.

---

# Observações para Desenvolvedores

Antes de modificar o código:

1. Verifique as filas FreeRTOS utilizadas entre tarefas.
2. Analise impactos na temporização das tarefas.
3. Mantenha compatibilidade com os formatos de telemetria já definidos.
4. Evite alterações nos protocolos de comunicação sem atualizar a estação base.
5. Certifique-se de que novas funcionalidades não bloqueiem as tarefas críticas.

Este documento foi elaborado para facilitar a manutenção e evolução do sistema por desenvolvedores externos.
