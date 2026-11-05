"""
saida_ssl.py

Manda a pose do robô e a posição da bola no formato da liga: SSL_WrapperPacket em
multicast 224.5.23.2:10020, o mesmo do SSL-Vision. A ponte_serial (--entrada ssl) e a IA
do ssl-robot leem isso direto.

O pacote é montado na mão, sem o pacote protobuf do Python: assim quem roda a visão não
precisa de protoc nem de acertar versão de biblioteca. São só os campos que usamos, com
os números de campo dos .proto em ../proto. O teste confere contra o protobuf de verdade.

Uso:

    from saida_ssl import PublicadorSSL

    pub = PublicadorSSL()                       # azul, id 0, multicast padrão
    t_captura, m0 = time.time(), time.monotonic()   # logo depois do cap.read()
    ...
    pub.enviar(t_captura, robo=(x_mm, y_mm, theta_rad), bola=(bx_mm, by_mm),
               processamento_s=time.monotonic() - m0)

Regras do contrato (INTERFACE_VISAO.md):
  - posição em mm e ângulo em rad, no referencial do campo (x para frente, y para a
    esquerda, theta anti-horário). A conversão de pixel vem da calibracao.py;
  - robô ou bola não detectados no frame: passe None. Não repita a última posição;
  - não arredonde a pose: com o robô parado, pose idêntica em frames seguidos é tratada
    como visão travada e a ponte freia.
"""

import socket
import struct
import time

GRUPO_PADRAO = "224.5.23.2"
PORTA_PADRAO = 10020
SSL_SOURCE_OTHER = 1


# ----------------------------------------------------------------- protobuf na mão

def _varint(n):
    saida = bytearray()
    while True:
        b = n & 0x7F
        n >>= 7
        if n:
            saida.append(b | 0x80)
        else:
            saida.append(b)
            return bytes(saida)


def _chave(campo, tipo):
    return _varint((campo << 3) | tipo)


def _uint(campo, valor):
    return _chave(campo, 0) + _varint(int(valor))


def _float(campo, valor):
    return _chave(campo, 5) + struct.pack("<f", float(valor))


def _double(campo, valor):
    return _chave(campo, 1) + struct.pack("<d", float(valor))


def _mensagem(campo, corpo):
    return _chave(campo, 2) + _varint(len(corpo)) + corpo


def _robo(rid, x, y, theta, confianca, px, py):
    # SSL_DetectionRobot: confidence=1, robot_id=2, x=3, y=4, orientation=5, pixel_x=6, pixel_y=7
    return (_float(1, confianca) + _uint(2, rid) + _float(3, x) + _float(4, y) +
            _float(5, theta) + _float(6, px) + _float(7, py))


def _bola(x, y, confianca, px, py):
    # SSL_DetectionBall: confidence=1, x=3, y=4, pixel_x=6, pixel_y=7
    return _float(1, confianca) + _float(3, x) + _float(4, y) + _float(6, px) + _float(7, py)


def montar_pacote(quadro, camera, t_captura, t_envio, robos_azuis=(), robos_amarelos=(), bolas=()):
    """
    Bytes de um SSL_WrapperPacket com uma deteção.
    robos_*: (id, x_mm, y_mm, theta_rad, confianca, pixel_x, pixel_y)
    bolas:   (x_mm, y_mm, confianca, pixel_x, pixel_y)
    """
    # SSL_DetectionFrame: frame_number=1, t_capture=2, t_sent=3, camera_id=4,
    # balls=5, robots_yellow=6, robots_blue=7
    frame = _uint(1, quadro) + _double(2, t_captura) + _double(3, t_envio) + _uint(4, camera)
    for b in bolas:
        frame += _mensagem(5, _bola(*b))
    for r in robos_amarelos:
        frame += _mensagem(6, _robo(*r))
    for r in robos_azuis:
        frame += _mensagem(7, _robo(*r))
    # SSL_WrapperPacket: detection=1, source=3
    return _mensagem(1, frame) + _uint(3, SSL_SOURCE_OTHER)


# ----------------------------------------------------------------------- envio

class PublicadorSSL:
    def __init__(self, time_azul=True, id_robo=0, camera=0,
                 grupo=GRUPO_PADRAO, porta=PORTA_PADRAO, interface="0.0.0.0"):
        self.time_azul = time_azul
        self.id_robo = id_robo
        self.camera = camera
        self.destino = (grupo, porta)
        self.quadro = 0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 1)
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)  # ponte no mesmo PC
        if interface != "0.0.0.0":
            self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF, socket.inet_aton(interface))

    def enviar(self, t_captura, robo=None, bola=None, confianca_robo=1.0, confianca_bola=1.0,
               pixel_robo=(0.0, 0.0), pixel_bola=(0.0, 0.0), processamento_s=None):
        """
        Manda um frame. t_captura: time.time() logo depois de ler a imagem da câmera.
        robo: (x_mm, y_mm, theta_rad) ou None. bola: (x_mm, y_mm) ou None.
        processamento_s: tempo desde a captura medido com time.monotonic(). Sem ele,
        t_sent é time.time() agora, e se o relógio for acertado no meio do frame (o WSL
        faz isso, deu -2 s no teste) a diferença sai errada.
        Frame sem nada também é mandado: a ponte sabe que a visão está viva, mas não vê o robô.
        """
        self.quadro += 1
        t_envio = time.time() if processamento_s is None else t_captura + processamento_s
        robos = []
        if robo is not None:
            robos = [(self.id_robo, robo[0], robo[1], robo[2], confianca_robo, *pixel_robo)]
        bolas = [(bola[0], bola[1], confianca_bola, *pixel_bola)] if bola is not None else []
        dados = montar_pacote(self.quadro, self.camera, t_captura, t_envio,
                              robos_azuis=robos if self.time_azul else (),
                              robos_amarelos=() if self.time_azul else robos,
                              bolas=bolas)
        self.sock.sendto(dados, self.destino)
        return dados

    def fechar(self):
        self.sock.close()
