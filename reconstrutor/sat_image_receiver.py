"""
Sat Image Receiver  (com GUI de debug)
======================================
Le a porta serial da estacao de solo (Heltec / leitor central / com0com),
detecta o protocolo de imagem:

    IMAGE_BEGIN
    SIZE:<n>
    <n bytes crus do JPEG>
    (linha em branco)
    IMAGE_END

monta o JPEG e exibe. A MESMA porta tambem carrega telemetria/debug em texto
(consumidos pelo AbaTrack) — este programa ignora esse texto para fins de
imagem, mas o MOSTRA na janela de debug para ajudar no diagnostico.

A logica de parsing vive em sat_protocol.ImageProtocolParser (testada por
tests/test_sat_protocol.py). Aqui ficam apenas a serial e a interface.

Compatibilidade: Windows 10/11. Pode rodar como .exe (PyInstaller --noconsole)
e se registrar para iniciar com o Windows.
"""

import sys
import os
import time
import threading
import queue
import io
from datetime import datetime

import serial
import serial.tools.list_ports
import tkinter as tk
from tkinter import ttk, filedialog

try:
    from sat_protocol import ImageProtocolParser
except ImportError:  # rodando de outro diretorio / empacotado
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from sat_protocol import ImageProtocolParser

# PIL e necessario para renderizar a imagem; se faltar, a GUI ainda funciona
# (mostra o log/eventos) mas sem preview/popup.
try:
    from PIL import Image, ImageTk
    HAS_PIL = True
except ImportError:
    HAS_PIL = False

# Recursos opcionais de Windows (autostart). Ausentes no Linux/macOS.
try:
    import winreg
    import shutil
    import ctypes
    TEM_WINDOWS = (os.name == "nt")
except ImportError:
    TEM_WINDOWS = False

# --------------------------------------------------------------------------
# CONFIG
# --------------------------------------------------------------------------
BAUD_RATE = 115200
RECONNECT_DELAY_S = 3
READ_TIMEOUT_S = 1.0
APP_NAME = "SatImageReceiver"
USB_HINTS = ["CP210", "CH9102", "CH340", "Silicon Labs", "USB-SERIAL", "USB Serial"]

# Porta de LEITURA dedicada deste consumidor (ponta com0com). Deixe None para
# auto-detectar a Heltec real. Ex.: r"\\.\COM15".
PORTA_FIXA = r"\\.\COM15"

# Mostra popups da imagem alem do preview na janela de debug.
MOSTRAR_POPUPS = True

INSTALL_DIR = os.path.join(os.environ.get("LOCALAPPDATA", os.path.expanduser("~")), APP_NAME)
INSTALL_PATH = os.path.join(INSTALL_DIR, f"{APP_NAME}.exe")

MAX_LOG_LINHAS = 500


# --------------------------------------------------------------------------
# AUTOSTART (registro do Windows) — inalterado
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
# DETECCAO DE PORTA
# --------------------------------------------------------------------------
def listar_portas():
    return [p.device for p in serial.tools.list_ports.comports()]


def escolher_porta():
    if PORTA_FIXA:
        return PORTA_FIXA
    portas = list(serial.tools.list_ports.comports())
    if not portas:
        return None
    for hint in USB_HINTS:
        for p in portas:
            descricao = f"{p.description} {p.manufacturer or ''}"
            if hint.lower() in descricao.lower():
                return p.device
    return portas[0].device


# --------------------------------------------------------------------------
# LEITURA SERIAL  (alimenta o ImageProtocolParser)
# --------------------------------------------------------------------------
class LeitorSerial(threading.Thread):
    """Mantem a serial aberta, reconectando, e empurra (canal, dado) na fila:
        ('status', texto)  -> mudancas de conexao
        ('event',  (tipo, dado)) -> eventos do parser (line/image/etc.)
    """

    def __init__(self, porta, baud, fila: queue.Queue):
        super().__init__(daemon=True)
        self.porta = porta
        self.baud = baud
        self.fila = fila
        self._parar = threading.Event()
        self.parser = ImageProtocolParser(on_event=self._on_event)

    def _on_event(self, tipo, dado):
        self.fila.put(("event", (tipo, dado)))

    def parar(self):
        self._parar.set()

    def run(self):
        while not self._parar.is_set():
            self.fila.put(("status", f"Conectando em {self.porta} @ {self.baud}..."))
            try:
                with serial.Serial(self.porta, self.baud, timeout=READ_TIMEOUT_S) as ser:
                    self.fila.put(("status", f"CONECTADO: {self.porta} @ {self.baud}"))
                    while not self._parar.is_set():
                        dados = ser.read(4096)
                        if dados:
                            self.parser.feed(dados)
            except (serial.SerialException, OSError) as e:
                self.fila.put(("status", f"Desconectado ({e}). Retentando..."))
                if self._parar.wait(RECONNECT_DELAY_S):
                    break
        self.fila.put(("status", "Leitura encerrada."))


# --------------------------------------------------------------------------
# GUI DE DEBUG
# --------------------------------------------------------------------------
class AppDebug:
    def __init__(self):
        self.fila: queue.Queue = queue.Queue()
        self.leitor = None
        self.ultima_imagem = None  # bytes do ultimo JPEG
        self._preview_ref = None   # mantem ref do PhotoImage (preview)
        self.popups = []

        self.root = tk.Tk()
        self.root.title("Sat Image Receiver — Debug")
        self.root.geometry("900x620")
        self.var_popups = tk.BooleanVar(value=MOSTRAR_POPUPS)

        self._build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self._ao_fechar_app)
        self._agendar_verificacao()

    # ----------------------------------------------------------------- UI
    def _build_ui(self):
        top = ttk.Frame(self.root, padding=6)
        top.pack(fill="x")

        ttk.Label(top, text="Porta:").pack(side="left")
        self.combo_porta = ttk.Combobox(top, width=18, values=listar_portas())
        if PORTA_FIXA:
            self.combo_porta.set(PORTA_FIXA)
        elif self.combo_porta["values"]:
            self.combo_porta.current(0)
        self.combo_porta.pack(side="left", padx=4)

        ttk.Button(top, text="↻", width=3, command=self._atualizar_portas).pack(side="left")

        ttk.Label(top, text="Baud:").pack(side="left", padx=(8, 0))
        self.combo_baud = ttk.Combobox(top, width=8,
                                       values=["115200", "9600", "57600", "38400"])
        self.combo_baud.set(str(BAUD_RATE))
        self.combo_baud.pack(side="left", padx=4)

        self.btn_conectar = ttk.Button(top, text="Conectar", command=self.conectar)
        self.btn_conectar.pack(side="left", padx=4)
        self.btn_desconectar = ttk.Button(top, text="Desconectar",
                                          command=self.desconectar, state="disabled")
        self.btn_desconectar.pack(side="left")

        ttk.Checkbutton(top, text="Popups", variable=self.var_popups).pack(side="left", padx=8)

        # Status
        self.lbl_status = ttk.Label(self.root, text="Aguardando conexao.",
                                    anchor="w", relief="sunken", padding=4)
        self.lbl_status.pack(fill="x", padx=6)

        # Stats
        stats = ttk.Frame(self.root, padding=(6, 2))
        stats.pack(fill="x")
        self.var_stats = {
            "Imagens OK": tk.StringVar(value="0"),
            "Erros img": tk.StringVar(value="0"),
            "Linhas txt": tk.StringVar(value="0"),
            "Ult. img (bytes)": tk.StringVar(value="-"),
            "Progresso": tk.StringVar(value="-"),
        }
        for nome, var in self.var_stats.items():
            cel = ttk.Frame(stats)
            cel.pack(side="left", padx=8)
            ttk.Label(cel, text=nome, foreground="#888").pack()
            ttk.Label(cel, textvariable=var, font=("TkDefaultFont", 11, "bold")).pack()

        # Corpo: log (esq) + preview (dir)
        corpo = ttk.Frame(self.root, padding=6)
        corpo.pack(fill="both", expand=True)

        esq = ttk.Frame(corpo)
        esq.pack(side="left", fill="both", expand=True)
        ttk.Label(esq, text="Eventos / dados crus:").pack(anchor="w")
        log_frame = ttk.Frame(esq)
        log_frame.pack(fill="both", expand=True)
        self.txt_log = tk.Text(log_frame, height=20, width=60, wrap="none",
                               bg="#101418", fg="#d0d0d0", insertbackground="#d0d0d0")
        scroll = ttk.Scrollbar(log_frame, command=self.txt_log.yview)
        self.txt_log.configure(yscrollcommand=scroll.set)
        scroll.pack(side="right", fill="y")
        self.txt_log.pack(side="left", fill="both", expand=True)
        self.txt_log.tag_config("img", foreground="#5fd35f")
        self.txt_log.tag_config("err", foreground="#ff6b6b")
        self.txt_log.tag_config("sys", foreground="#6bb6ff")

        dir_ = ttk.Frame(corpo)
        dir_.pack(side="right", fill="y", padx=(8, 0))
        ttk.Label(dir_, text="Ultima imagem:").pack(anchor="w")
        self.lbl_preview = ttk.Label(dir_, text="(sem imagem)", width=40,
                                     anchor="center", relief="groove")
        self.lbl_preview.pack(pady=4)
        ttk.Button(dir_, text="Salvar imagem...", command=self.salvar_imagem).pack(fill="x")
        ttk.Button(dir_, text="Limpar log", command=lambda: self.txt_log.delete("1.0", "end")).pack(fill="x", pady=4)

        if not HAS_PIL:
            self._log("PIL/Pillow ausente: preview e popups desativados "
                      "(instale 'pillow' no Windows).", "err")

    def _atualizar_portas(self):
        self.combo_porta["values"] = listar_portas()

    # ------------------------------------------------------------- conexao
    def conectar(self):
        porta = self.combo_porta.get().strip()
        if not porta:
            self._log("Nenhuma porta selecionada.", "err")
            return
        try:
            baud = int(self.combo_baud.get())
        except ValueError:
            baud = BAUD_RATE
        self.leitor = LeitorSerial(porta, baud, self.fila)
        self.leitor.start()
        self.btn_conectar.configure(state="disabled")
        self.btn_desconectar.configure(state="normal")

    def desconectar(self):
        if self.leitor:
            self.leitor.parar()
            self.leitor = None
        self.btn_conectar.configure(state="normal")
        self.btn_desconectar.configure(state="disabled")

    # --------------------------------------------------------------- fila
    def _agendar_verificacao(self):
        self._verificar_fila()
        self.root.after(150, self._agendar_verificacao)

    def _verificar_fila(self):
        try:
            while True:
                canal, dado = self.fila.get_nowait()
                if canal == "status":
                    self.lbl_status.configure(text=dado)
                    self._log(dado, "sys")
                elif canal == "event":
                    self._tratar_evento(*dado)
        except queue.Empty:
            pass

    def _tratar_evento(self, tipo, dado):
        if tipo == "line":
            self._log(f"TXT  {dado}")
            self.var_stats["Linhas txt"].set(str(int(self.var_stats["Linhas txt"].get()) + 1))
        elif tipo == "image_begin":
            self._log(">>> IMAGE_BEGIN", "img")
        elif tipo == "size":
            self._log(f"    SIZE = {dado} bytes", "img")
            self.var_stats["Progresso"].set(f"0/{dado}")
        elif tipo == "progress":
            got, tot = dado
            self.var_stats["Progresso"].set(f"{got}/{tot}")
        elif tipo == "image_complete":
            n = len(dado)
            self._log(f"<<< IMAGEM OK ({n} bytes)", "img")
            self.var_stats["Imagens OK"].set(str(int(self.var_stats["Imagens OK"].get()) + 1))
            self.var_stats["Ult. img (bytes)"].set(str(n))
            self.var_stats["Progresso"].set("completo")
            self._mostrar_imagem(dado)
        elif tipo == "image_error":
            self._log(f"!!! {dado}", "err")
            self.var_stats["Erros img"].set(str(int(self.var_stats["Erros img"].get()) + 1))

    def _log(self, texto, tag=None):
        ts = datetime.now().strftime("%H:%M:%S")
        self.txt_log.insert("end", f"[{ts}] {texto}\n", tag or ())
        # Limita o tamanho do log.
        linhas = int(self.txt_log.index("end-1c").split(".")[0])
        if linhas > MAX_LOG_LINHAS:
            self.txt_log.delete("1.0", f"{linhas - MAX_LOG_LINHAS}.0")
        self.txt_log.see("end")

    # -------------------------------------------------------------- imagem
    def _mostrar_imagem(self, dados_jpeg: bytes):
        self.ultima_imagem = dados_jpeg
        if not HAS_PIL:
            return
        try:
            img = Image.open(io.BytesIO(dados_jpeg))
            img.load()
        except Exception:
            self._log("JPEG corrompido — nao renderizado.", "err")
            return

        # Preview na janela principal (lado direito).
        prev = img.copy()
        prev.thumbnail((320, 320), Image.LANCZOS)
        self._preview_ref = ImageTk.PhotoImage(prev)
        self.lbl_preview.configure(image=self._preview_ref, text="")

        if self.var_popups.get():
            self._popup(img)

    def _popup(self, img):
        largura, altura = img.size
        max_w, max_h = 1000, 800
        escala = min(max_w / largura, max_h / altura, 1.0)
        if escala < 1.0:
            img = img.resize((int(largura * escala), int(altura * escala)), Image.LANCZOS)

        janela = tk.Toplevel(self.root)
        janela.title(f"Imagem recebida ({largura}x{altura})")
        janela.attributes("-topmost", True)
        foto = ImageTk.PhotoImage(img)
        label = tk.Label(janela, image=foto)
        label.image = foto
        label.pack()

        def ao_fechar():
            if janela in self.popups:
                self.popups.remove(janela)
            janela.destroy()

        janela.protocol("WM_DELETE_WINDOW", ao_fechar)
        self.popups.append(janela)

    def salvar_imagem(self):
        if not self.ultima_imagem:
            self._log("Nenhuma imagem para salvar.", "err")
            return
        nome = f"sat_image_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
        path = filedialog.asksaveasfilename(
            defaultextension=".jpg", initialfile=nome,
            filetypes=[("JPEG", "*.jpg"), ("Todos", "*.*")])
        if path:
            try:
                with open(path, "wb") as f:
                    f.write(self.ultima_imagem)
                self._log(f"Imagem salva: {path}", "img")
            except Exception as e:
                self._log(f"Erro ao salvar: {e}", "err")

    # ---------------------------------------------------------------- ciclo
    def _ao_fechar_app(self):
        self.desconectar()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main():
    if TEM_WINDOWS:
        try:
            ctypes.windll.kernel32.CreateMutexW(None, False, f"Mutex_{APP_NAME}")
            if ctypes.windll.kernel32.GetLastError() == 183:  # ERROR_ALREADY_EXISTS
                sys.exit(0)
        except Exception:
            pass

    instalar_autostart_e_copiar()

    app = AppDebug()
    # Auto-conecta se ja houver uma porta definida/selecionada.
    if app.combo_porta.get().strip():
        app.conectar()
    app.run()


if __name__ == "__main__":
    main()
