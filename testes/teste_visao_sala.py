#!/usr/bin/env python3
"""
teste_visao_sala.py

Testa a visão (visao/) sem câmera, com imagens sintéticas de pose conhecida: uma câmera
inclinada vendo o chão, o robô com os três círculos (F azul, E vermelho, D preto) e a
bola laranja, com ruído de câmera.

    .venv/bin/python testes/teste_visao_sala.py

Confere:
  1. o pacote montado na mão (saida_ssl.py) contra o protobuf de verdade;
  2. a calibração por homografia, com pontos medidos com erro;
  3. a detecção: erro de posição e ângulo do robô e da bola ao longo de uma trajetória;
  4. robô parado: a pose não pode se repetir igual (a ponte trataria como visão travada);
  5. círculo da frente coberto: robô não detectado, e não a última pose;
  6. ponta a ponta: vídeo -> visao_sala.py -> multicast -> leitura com o protobuf.

Precisa de numpy, opencv e protobuf, e do protoc para gerar o módulo de leitura.
Devolve 0 se tudo passa.
"""

import math
import os
import socket
import struct
import subprocess
import sys
import tempfile
import time

import cv2
import numpy as np

RAIZ = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, os.path.join(RAIZ, "visao"))
from calibracao import Calibracao, homografia   # noqa: E402
from deteccao import CONFIG_PADRAO, detectar_bola, detectar_robo  # noqa: E402
from saida_ssl import montar_pacote             # noqa: E402

LARGURA, ALTURA = 1280, 720
RAIO_PX = 45            # círculo da marcação na imagem
DIST_CIRCULO_MM = 150   # do centro do robô ao centro de cada círculo (> minDist do Hough)
RAIO_BOLA_PX = 14

# Câmera inclinada: onde os cantos de uma área de 2,0 x 1,2 m caem na imagem (trapézio)
CANTOS_CAMPO = [(-1000, 600), (1000, 600), (1000, -600), (-1000, -600)]
CANTOS_PIXEL = [(150, 40), (1130, 40), (1260, 700), (20, 700)]
H_CAMPO_PIXEL = homografia(CANTOS_CAMPO, CANTOS_PIXEL)

falhas = 0


def conferir(ok, nome):
    global falhas
    print(f"  {nome:60s} {'ok' if ok else 'FALHOU'}")
    if not ok:
        falhas += 1


def para_pixel(x, y):
    u, v, w = H_CAMPO_PIXEL @ np.array([x, y, 1.0])
    return u / w, v / w


CHAO_CINZA = (110, 120, 115)
CHAO_CARPETE = (40, 90, 35)    # verde escuro, como o campo da SSL


def desenhar(x, y, theta, bola=None, sem_frente=False, semente=0, chao=CHAO_CINZA):
    """Quadro BGR com o robô na pose (x, y, theta) do campo e a bola."""
    rng = np.random.default_rng(semente)
    img = np.full((ALTURA, LARGURA, 3), chao, np.uint8)
    cores = {0.0: (220, 90, 20), 2 * math.pi / 3: (30, 30, 210), -2 * math.pi / 3: (25, 25, 25)}
    for desvio, cor in cores.items():
        if sem_frente and desvio == 0.0:
            continue
        cx = x + DIST_CIRCULO_MM * math.cos(theta + desvio)
        cy = y + DIST_CIRCULO_MM * math.sin(theta + desvio)
        u, v = para_pixel(cx, cy)
        # desenha com subpixel (shift 4) para a posição não ficar presa ao pixel
        cv2.circle(img, (int(u * 16), int(v * 16)), RAIO_PX * 16, cor, -1, cv2.LINE_AA, 4)
    if bola is not None:
        u, v = para_pixel(*bola)
        cv2.circle(img, (int(u * 16), int(v * 16)), RAIO_BOLA_PX * 16, (0, 120, 255), -1,
                   cv2.LINE_AA, 4)
    ruido = rng.normal(0, 3.0, img.shape)
    return np.clip(img + ruido, 0, 255).astype(np.uint8)


def centro_robo_campo(x, y, theta):
    """Centro dos três círculos no campo (é o x, y desenhado) e a frente em theta."""
    return x, y, theta


def trajetoria(n):
    """Robô andando numa elipse e girando; bola parada."""
    for i in range(n):
        a = 2 * math.pi * i / n
        yield 500 * math.cos(a), 250 * math.sin(a), math.atan2(math.sin(a), math.cos(a)) + 0.6


def calibracao_medida():
    """Calibração como seria feita com trena: 8 pontos com 0,7 px e 2 mm de erro."""
    rng = np.random.default_rng(7)
    campo = [(-900, 500), (0, 500), (900, 500), (900, 0), (900, -500), (0, -500),
             (-900, -500), (-900, 0)]
    pixels = [tuple(np.add(para_pixel(*c), rng.normal(0, 0.7, 2))) for c in campo]
    medidos = [tuple(np.add(c, rng.normal(0, 2.0, 2))) for c in campo]
    return Calibracao.de_pontos(pixels, medidos), pixels, medidos


def angulo(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def pb_ssl():
    saida = tempfile.mkdtemp(prefix="proto_ssl_")
    subprocess.run(["protoc", f"--proto_path={os.path.join(RAIZ, 'proto')}", f"--python_out={saida}",
                    "messages_robocup_ssl_detection.proto", "messages_robocup_ssl_geometry.proto",
                    "messages_robocup_ssl_wrapper.proto"], check=True)
    sys.path.insert(0, saida)
    import messages_robocup_ssl_wrapper_pb2
    return messages_robocup_ssl_wrapper_pb2


def teste_pacote(pb):
    print("pacote SSL montado na mao")
    dados = montar_pacote(42, 1, 1000.25, 1000.254,
                          robos_azuis=[(0, 1234.5, -678.25, -2.5, 0.9, 10.5, 20.25)],
                          bolas=[(50.5, -60.5, 0.8, 1.0, 2.0)])
    p = pb.SSL_WrapperPacket()
    p.ParseFromString(dados)
    d = p.detection
    r = d.robots_blue[0]
    conferir(p.IsInitialized() and d.frame_number == 42 and d.camera_id == 1,
             "abre no protobuf, com todos os campos obrigatorios")
    conferir(abs(d.t_sent - d.t_capture - 0.004) < 1e-9, "t_capture e t_sent em double")
    conferir(r.robot_id == 0 and r.x == 1234.5 and r.y == -678.25 and abs(r.orientation + 2.5) < 1e-6,
             "robo azul 0 com x, y e orientation")
    conferir(len(d.robots_yellow) == 0 and d.balls[0].x == 50.5, "bola, e nenhum amarelo")

    vazio = pb.SSL_WrapperPacket()
    vazio.ParseFromString(montar_pacote(1, 0, 1.0, 1.0))
    conferir(vazio.IsInitialized() and not vazio.detection.robots_blue and not vazio.detection.balls,
             "frame sem robo e sem bola continua valido")


def teste_calibracao():
    print("\ncalibracao")
    cal, pixels, medidos = calibracao_medida()
    erro_pts = cal.erro_rms(pixels, medidos)
    grade = [(x, y) for x in range(-900, 901, 150) for y in range(-500, 501, 125)]
    erros = [math.hypot(*np.subtract(cal.pixel_para_campo(*para_pixel(x, y)), (x, y))) for x, y in grade]
    print(f"  erro nos pontos de calibracao {erro_pts:.1f} mm, na area toda: "
          f"medio {np.mean(erros):.1f} mm, pior {max(erros):.1f} mm")
    conferir(max(erros) < 6.0, "pixel -> mm com erro abaixo de 6 mm na area toda")
    th = 1.0
    u0, v0 = para_pixel(200, 100)
    u1, v1 = para_pixel(200 + 50 * math.cos(th), 100 + 50 * math.sin(th))
    conferir(abs(angulo(cal.angulo_para_campo(u0, v0, math.atan2(v1 - v0, u1 - u0)) - th)) < 0.01,
             "angulo da imagem (y para baixo) vira theta anti-horario no campo")
    return cal


def teste_deteccao(cal, chao=CHAO_CINZA, nome_chao="chao cinza"):
    print(f"\ndeteccao, {nome_chao} (trajetoria de 60 quadros, camera inclinada, ruido de 3 niveis)")
    bola = (650.0, -300.0)
    erros_pos, erros_ang, erros_bola, perdidos = [], [], [], 0
    for i, (x, y, th) in enumerate(trajetoria(60)):
        frame = desenhar(x, y, th, bola, semente=i, chao=chao)
        r = detectar_robo(frame)
        b = detectar_bola(frame)
        if r is None:
            perdidos += 1
            continue
        xm, ym = cal.pixel_para_campo(r[0], r[1])
        erros_pos.append(math.hypot(xm - x, ym - y))
        erros_ang.append(abs(angulo(cal.angulo_para_campo(r[0], r[1], r[2]) - th)))
        if b is not None:
            erros_bola.append(math.hypot(*np.subtract(cal.pixel_para_campo(*b), bola)))
    rms = lambda e: math.sqrt(sum(v * v for v in e) / len(e)) if e else float("inf")  # noqa: E731
    print(f"  robo: {60 - perdidos}/60 quadros, posicao RMS {rms(erros_pos):.1f} mm, "
          f"angulo RMS {math.degrees(rms(erros_ang)):.2f} graus")
    print(f"  bola: {len(erros_bola)}/{60 - perdidos} quadros, posicao RMS {rms(erros_bola):.1f} mm")
    conferir(perdidos == 0, "robo detectado em todos os quadros")
    conferir(rms(erros_pos) < 8.0, "posicao do robo com erro RMS abaixo de 8 mm")
    conferir(math.degrees(rms(erros_ang)) < 2.0, "angulo do robo com erro RMS abaixo de 2 graus")
    conferir(len(erros_bola) == 60 - perdidos and rms(erros_bola) < 8.0, "bola detectada, erro abaixo de 8 mm")

    poses = []
    for i in range(10):
        r = detectar_robo(desenhar(100, 50, 0.3, bola, semente=1000 + i, chao=chao))
        poses.append((r[0], r[1], r[2]) if r else None)
    iguais = sum(1 for a, b in zip(poses, poses[1:]) if a is not None and a == b)
    espalhamento = np.std([p[0] for p in poses if p]) if all(poses) else float("inf")
    print(f"  robo parado: {iguais} pares de quadros iguais em 9, desvio {espalhamento:.3f} px")
    conferir(all(poses) and iguais == 0, "robo parado: pose nunca se repete igual (nao parece travada)")

    sem_f = detectar_robo(desenhar(100, 50, 0.3, bola, sem_frente=True, semente=5, chao=chao))
    conferir(sem_f is None, "circulo da frente coberto: robo nao detectado")

    # Sombra comprida ao lado do robô: não é redonda, tem que ser ignorada
    frame = desenhar(100, 50, 0.3, bola, semente=6, chao=chao)
    u, v = para_pixel(100, -250)
    cv2.rectangle(frame, (int(u) - 200, int(v) - 25), (int(u) + 200, int(v) + 25), (20, 20, 20), -1)
    r = detectar_robo(frame)
    ok = r is not None and math.hypot(*np.subtract(cal.pixel_para_campo(r[0], r[1]), (100, 50))) < 10
    conferir(ok, "sombra preta comprida ao lado: robo detectado no lugar certo")

    # Objeto preto redondo maior, longe: não pode virar uma pose errada
    frame = desenhar(100, 50, 0.3, bola, semente=7, chao=chao)
    u, v = para_pixel(-800, -450)
    cv2.circle(frame, (int(u), int(v)), 70, (15, 15, 15), -1)
    r = detectar_robo(frame)
    ok = r is None or math.hypot(*np.subtract(cal.pixel_para_campo(r[0], r[1]), (100, 50))) < 10
    conferir(ok, "objeto preto redondo longe: nao inventa pose (nada ou a certa)")


def teste_ponta_a_ponta(cal, pb):
    print("\nponta a ponta: video -> visao_sala.py -> multicast")
    pasta = tempfile.mkdtemp(prefix="visao_sala_")
    video = os.path.join(pasta, "trajetoria.avi")
    caminho_cal = os.path.join(pasta, "calibracao.json")
    cal.salvar(caminho_cal)

    bola = (650.0, -300.0)
    poses = list(trajetoria(90))
    gravador = cv2.VideoWriter(video, cv2.VideoWriter_fourcc(*"MJPG"), 30, (LARGURA, ALTURA))
    for i, (x, y, th) in enumerate(poses):
        # os 10 últimos sem a frente: a visão tem que parar de mandar o robô
        gravador.write(desenhar(x, y, th, bola, sem_frente=i >= 80, semente=i))
    gravador.release()

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("", 10020))
    s.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                 struct.pack("4s4s", socket.inet_aton("224.5.23.2"), socket.inet_aton("0.0.0.0")))
    s.settimeout(0.5)

    proc = subprocess.Popen([sys.executable, os.path.join(RAIZ, "visao", "visao_sala.py"),
                             "--fonte", video, "--calibracao", caminho_cal, "--sem-janela"],
                            stdout=subprocess.DEVNULL)
    recebidos = {}
    fim = time.time() + 15
    while time.time() < fim and len(recebidos) < len(poses):
        try:
            dados = s.recv(65536)
        except socket.timeout:
            if proc.poll() is not None:
                break
            continue
        p = pb.SSL_WrapperPacket()
        p.ParseFromString(dados)
        if p.HasField("detection") and p.source == pb.SSL_SOURCE_OTHER:
            recebidos[p.detection.frame_number] = p.detection
    proc.wait(timeout=10)
    s.close()

    erros, procs = [], []
    for n, d in recebidos.items():
        procs.append(d.t_sent - d.t_capture)
        if n <= 80 and d.robots_blue:
            x, y, _ = poses[n - 1]
            erros.append(math.hypot(d.robots_blue[0].x - x, d.robots_blue[0].y - y))
    sem_robo_fim = all(not recebidos[n].robots_blue for n in recebidos if n > 80)
    rms = math.sqrt(sum(e * e for e in erros) / len(erros)) if erros else float("inf")
    print(f"  {len(recebidos)}/{len(poses)} pacotes, robo em {len(erros)}/80, erro RMS {rms:.1f} mm "
          f"(com compressao MJPG), processamento medio {1000 * np.mean(procs):.1f} ms")
    conferir(len(recebidos) == len(poses), "um pacote por quadro do video")
    conferir(len(erros) >= 78 and rms < 10.0, "robo no pacote com erro RMS abaixo de 10 mm")
    conferir(sem_robo_fim, "sem a frente, o pacote vai sem o robo (nao repete a ultima pose)")
    conferir(all(0 <= t < 0.2 for t in procs), "t_sent - t_capture positivo e pequeno")


def main():
    pb = pb_ssl()
    teste_pacote(pb)
    cal = teste_calibracao()
    teste_deteccao(cal)
    teste_deteccao(cal, CHAO_CARPETE, "carpete verde escuro")
    teste_ponta_a_ponta(cal, pb)
    print(f"\n{'HA TESTES FALHANDO' if falhas else 'todos os testes passaram'}")
    return 1 if falhas else 0


if __name__ == "__main__":
    sys.exit(main())
