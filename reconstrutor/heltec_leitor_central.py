"""
Leitor Central (HUB) - Heltec -> N portas COM virtuais (com0com)
================================================================
Resolve "duas (ou mais) aplicacoes nao podem abrir a mesma porta COM real
ao mesmo tempo no Windows".

Este programa:
  1. Abre a porta COM REAL da Heltec (auto-detectada).
  2. Faz FAN-OUT: copia, byte a byte, tudo que chega dela para VARIAS portas
     COM virtuais de escrita criadas pelo com0com (uma por par).
  3. Cada par com0com (ex: COM12<->COM13, COM14<->COM15) entrega os mesmos
     bytes na ponta de leitura. Assim CADA consumidor tem sua propria porta
     dedicada:
         COM12 (escrevemos) -> COM13 (lida pelo AbaTrack)
         COM14 (escrevemos) -> COM15 (lida pelo sat_image_receiver.py)

Por que fan-out em vez de uma porta so?
  Mesmo virtual, uma porta COM normalmente so aceita UM processo abrindo-a
  para leitura. Duplicando o stream para dois pares, cada programa abre a
  SUA porta sem disputa.

PRE-REQUISITO (com0com "Setup Command Prompt"), criar DOIS pares:
    install PortName=COM12 PortName=COM13
    install PortName=COM14 PortName=COM15

Configure abaixo COM_VIRTUAIS_ESCRITA com as pontas que ESTE hub escreve
(ex: COM12 e COM14). Os consumidores leem as OUTRAS pontas (COM13, COM15).

Para voltar ao modo antigo (um unico consumidor), deixe so uma porta na lista.
Roda sem console (--noconsole) e pode iniciar com o Windows, igual antes.
"""

import sys
import os
import time
import threading

import serial
import serial.tools.list_ports

try:
    import ctypes
    import shutil
    import winreg
    TEM_WINDOWS = (os.name == "nt")
except ImportError:
    TEM_WINDOWS = False

# --------------------------------------------------------------------------
# CONFIG - AJUSTE AQUI
# --------------------------------------------------------------------------
BAUD_RATE = 115200
RECONNECT_DELAY_S = 3
READ_TIMEOUT_S = 1.0

# Pontas de ESCRITA do hub (uma por par com0com). Os consumidores leem as
# pontas opostas. Deixe uma so para o comportamento antigo de consumidor unico.
COM_VIRTUAIS_ESCRITA = [r"\\.\COM12", r"\\.\COM14"]

# Portas a IGNORAR ao procurar a Heltec real (as virtuais do com0com).
IGNORAR_PORTAS = {"COM12", "COM13", "COM14", "COM15"}

USB_HINTS = ["CP210", "CH9102", "CH340", "Silicon Labs", "USB-SERIAL", "USB Serial"]

APP_NAME = "HeltecLeitorCentral"
INSTALL_DIR = os.path.join(os.environ.get("LOCALAPPDATA", os.path.expanduser("~")), APP_NAME)
INSTALL_PATH = os.path.join(INSTALL_DIR, f"{APP_NAME}.exe")


# --------------------------------------------------------------------------
# AUTOSTART (inalterado)
# --------------------------------------------------------------------------
def instalar_autostart_e_copiar():
    if not TEM_WINDOWS or not getattr(sys, "frozen", False):
        return

    exe_atual = sys.executable
    exe_atual_norm = os.path.normcase(os.path.abspath(exe_atual))
    instalado_norm = os.path.normcase(os.path.abspath(INSTALL_PATH))
    if exe_atual_norm == instalado_norm:
        return

    try:
        os.makedirs(INSTALL_DIR, exist_ok=True)
        shutil.copy2(exe_atual, INSTALL_PATH)
    except Exception:
        pass

    caminho = INSTALL_PATH if os.path.exists(INSTALL_PATH) else exe_atual
    try:
        chave = winreg.OpenKey(
            winreg.HKEY_CURRENT_USER,
            r"Software\Microsoft\Windows\CurrentVersion\Run",
            0, winreg.KEY_SET_VALUE,
        )
        winreg.SetValueEx(chave, APP_NAME, 0, winreg.REG_SZ, f'"{caminho}"')
        winreg.CloseKey(chave)
    except Exception:
        pass

    if os.path.exists(INSTALL_PATH) and exe_atual_norm != instalado_norm:
        try:
            os.startfile(INSTALL_PATH)
        except Exception:
            pass
        else:
            sys.exit(0)


# --------------------------------------------------------------------------
# DETECCAO DA PORTA REAL DA HELTEC
# --------------------------------------------------------------------------
def escolher_porta_heltec():
    """Procura a porta real da Heltec, ignorando as portas virtuais com0com."""
    portas = list(serial.tools.list_ports.comports())

    portas_reais = []
    for p in portas:
        desc = f"{p.description} {p.manufacturer or ''}".lower()
        if "com0com" in desc:
            continue
        if p.device.upper() in IGNORAR_PORTAS:
            continue
        portas_reais.append(p)

    if not portas_reais:
        return None

    for hint in USB_HINTS:
        for p in portas_reais:
            desc = f"{p.description} {p.manufacturer or ''}".lower()
            if hint.lower() in desc:
                return p.device
    return portas_reais[0].device


# --------------------------------------------------------------------------
# ABERTURA DAS PORTAS VIRTUAIS DE ESCRITA
# --------------------------------------------------------------------------
def abrir_virtuais():
    """Tenta abrir todas as pontas de escrita. Retorna lista de Serial abertas
    (pode ser parcial). Nao falha se uma porta especifica nao existir."""
    abertas = []
    for nome in COM_VIRTUAIS_ESCRITA:
        try:
            abertas.append(serial.Serial(nome, BAUD_RATE, timeout=READ_TIMEOUT_S))
        except (serial.SerialException, OSError):
            # Par com0com ausente/ocupado — segue com as demais.
            pass
    return abertas


def fechar_todas(portas):
    for s in portas:
        try:
            s.close()
        except Exception:
            pass


# --------------------------------------------------------------------------
# PONTE: HELTEC (real) -> N VIRTUAIS  (fan-out)
# --------------------------------------------------------------------------
def rodar_ponte():
    while True:
        porta_heltec = escolher_porta_heltec()
        if porta_heltec is None:
            time.sleep(RECONNECT_DELAY_S)
            continue

        try:
            ser_heltec = serial.Serial(porta_heltec, BAUD_RATE, timeout=READ_TIMEOUT_S)
        except (serial.SerialException, OSError):
            time.sleep(RECONNECT_DELAY_S)
            continue

        virtuais = abrir_virtuais()
        if not virtuais:
            # Sem nenhuma ponta de saida, nao ha pra onde copiar.
            ser_heltec.close()
            time.sleep(RECONNECT_DELAY_S)
            continue

        try:
            _loop_copia(ser_heltec, virtuais)
        except (serial.SerialException, OSError):
            pass
        finally:
            fechar_todas([ser_heltec] + virtuais)

        time.sleep(RECONNECT_DELAY_S)


def _loop_copia(ser_heltec, virtuais):
    """Le bytes crus da Heltec e replica em todas as virtuais. Se a escrita
    falhar numa virtual, descarta SO ela e continua com as demais. Se todas
    cairem, levanta para reconectar tudo."""
    while True:
        dados = ser_heltec.read(4096)
        if not dados:
            continue
        vivas = []
        for s in virtuais:
            try:
                s.write(dados)
                vivas.append(s)
            except (serial.SerialException, OSError):
                try:
                    s.close()
                except Exception:
                    pass
        virtuais[:] = vivas
        if not virtuais:
            raise serial.SerialException("Todas as portas virtuais cairam")


# --------------------------------------------------------------------------
# MAIN
# --------------------------------------------------------------------------
def main():
    if TEM_WINDOWS:
        try:
            ctypes.windll.kernel32.CreateMutexW(None, False, f"Mutex_{APP_NAME}")
            if ctypes.windll.kernel32.GetLastError() == 183:  # ERROR_ALREADY_EXISTS
                sys.exit(0)
        except Exception:
            pass

    instalar_autostart_e_copiar()
    rodar_ponte()


if __name__ == "__main__":
    main()
