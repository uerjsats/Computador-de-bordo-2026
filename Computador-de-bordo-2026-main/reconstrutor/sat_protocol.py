"""
sat_protocol.py
===============
Parser puro (sem dependencia de pyserial/tkinter) do protocolo de imagem
emitido pela estacao de solo na porta serial:

    IMAGE_BEGIN
    SIZE:<n>
    <n bytes crus do JPEG>
    (linha em branco)
    IMAGE_END

A mesma porta tambem carrega telemetria e debug em TEXTO, intercalados entre
imagens. Por isso o parser:

  * Le os <n> bytes da imagem por CONTAGEM (nao por linha) — assim bytes como
    '\\n', '\\r' ou ate a string "IMAGE_END" dentro do JPEG nao quebram nada.
  * Emite as linhas de texto que NAO fazem parte de uma imagem como eventos
    'line' (telemetria/debug) — o consumidor decide o que fazer com elas.
  * Aceita bytes em pedacos arbitrarios (alimentacao incremental via feed()).

Eventos (callback on_event(tipo, dado)):
    ('line', str)            linha de texto fora de imagem (telemetria/debug)
    ('image_begin', None)    detectou IMAGE_BEGIN
    ('size', int)            tamanho anunciado da imagem
    ('progress', (got,tot))  progresso de recepcao dos bytes da imagem
    ('image_complete', bytes) imagem completa e validada
    ('image_error', str)     algo invalido; parser volta a procurar

Testado por tests/test_sat_protocol.py (somente stdlib).
"""

# Limite de sanidade: rejeita SIZE absurdo (protege contra ruido/dessincronia).
MAX_IMAGE_SIZE = 200000

_SEARCHING = "SEARCHING"   # procurando IMAGE_BEGIN (ou repassando texto)
_WAIT_SIZE = "WAIT_SIZE"   # aguardando linha SIZE:<n>
_READ_BYTES = "READ_BYTES"  # lendo n bytes crus
_WAIT_END = "WAIT_END"     # aguardando IMAGE_END (ignorando linhas em branco)


class ImageProtocolParser:
    def __init__(self, on_event=None):
        self.on_event = on_event
        self._buf = bytearray()
        self.state = _SEARCHING
        self._expected = 0
        self._img = bytearray()
        # Estatisticas uteis para a GUI de debug.
        self.stats = {
            "images_ok": 0,
            "image_errors": 0,
            "lines": 0,
            "last_image_size": 0,
            "bytes_in_progress": 0,
            "expected_size": 0,
        }

    # ------------------------------------------------------------------ utils
    def _emit(self, tipo, dado=None):
        if self.on_event is not None:
            self.on_event(tipo, dado)

    def _reset_imagem(self):
        self._expected = 0
        self._img = bytearray()
        self.stats["bytes_in_progress"] = 0
        self.stats["expected_size"] = 0
        self.state = _SEARCHING

    def _pop_line(self):
        """Remove e devolve a proxima linha (ate '\\n') decodificada e limpa,
        ou None se ainda nao ha uma linha completa no buffer."""
        idx = self._buf.find(b"\n")
        if idx == -1:
            return None
        raw = self._buf[:idx + 1]
        del self._buf[:idx + 1]
        return raw.decode("utf-8", errors="ignore").strip()

    # ------------------------------------------------------------------- feed
    def feed(self, data: bytes):
        """Alimenta bytes; devolve lista de imagens (bytes) completadas agora."""
        if data:
            self._buf.extend(data)
        completas = []

        avancou = True
        while avancou:
            avancou = False

            if self.state == _READ_BYTES:
                faltam = self._expected - len(self._img)
                if faltam > 0:
                    if not self._buf:
                        break
                    pega = self._buf[:faltam]
                    del self._buf[:len(pega)]
                    self._img.extend(pega)
                    self.stats["bytes_in_progress"] = len(self._img)
                    self._emit("progress", (len(self._img), self._expected))
                    avancou = True
                if len(self._img) >= self._expected:
                    self.state = _WAIT_END
                    avancou = True
                continue

            # Estados baseados em linha: SEARCHING / WAIT_SIZE / WAIT_END
            linha = self._pop_line()
            if linha is None:
                break
            avancou = True

            if self.state == _SEARCHING:
                if linha == "IMAGE_BEGIN":
                    self._img = bytearray()
                    self.state = _WAIT_SIZE
                    self._emit("image_begin")
                elif linha != "":
                    self.stats["lines"] += 1
                    self._emit("line", linha)

            elif self.state == _WAIT_SIZE:
                if linha.startswith("SIZE:"):
                    try:
                        n = int(linha[5:])
                    except ValueError:
                        n = -1
                    if 0 < n <= MAX_IMAGE_SIZE:
                        self._expected = n
                        self.stats["expected_size"] = n
                        self.state = _READ_BYTES
                        self._emit("size", n)
                    else:
                        self.stats["image_errors"] += 1
                        self._emit("image_error", f"SIZE invalido: {linha}")
                        self._reset_imagem()
                else:
                    self.stats["image_errors"] += 1
                    self._emit("image_error", f"esperava SIZE, veio: {linha}")
                    self._reset_imagem()
                    # A linha inesperada pode ser texto util; nao a descarta.
                    if linha and linha != "IMAGE_BEGIN":
                        self.stats["lines"] += 1
                        self._emit("line", linha)

            elif self.state == _WAIT_END:
                if linha == "IMAGE_END":
                    img = bytes(self._img)
                    self.stats["images_ok"] += 1
                    self.stats["last_image_size"] = len(img)
                    completas.append(img)
                    self._emit("image_complete", img)
                    self._reset_imagem()
                elif linha == "":
                    pass  # linha em branco esperada antes do IMAGE_END
                else:
                    self.stats["image_errors"] += 1
                    self._emit("image_error", f"esperava IMAGE_END, veio: {linha}")
                    self._reset_imagem()
                    if linha != "IMAGE_BEGIN":
                        self.stats["lines"] += 1
                        self._emit("line", linha)

        return completas
