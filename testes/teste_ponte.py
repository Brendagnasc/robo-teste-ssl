#!/usr/bin/env python3
"""
teste_ponte.py

Testa a ponte_serial sem robô e sem câmera: cria uma porta serial virtual (pty),
simula a visão mandando pose por UDP e mostra os quadros que chegariam ao Arduino.

    cmake -B build && cmake --build build
    python3 testes/teste_ponte.py ./build/ponte_serial
    python3 testes/teste_ponte.py ./build/ponte_serial --ssl

O simulador integra a velocidade comandada, então a pose "anda" e a FSM fecha o ciclo
inteiro: ALINHAMENTO, DESLOCAMENTO, FRENAGEM, PARADO.

Com --ssl a visão sai como SSL_WrapperPacket no multicast 224.5.23.2:10020, igual ao
SSL-Vision: duas câmeras, a bola só numa delas, o robô amarelo de mesmo id na cena e uma
deteção falsa de baixa confiança. Precisa do protoc para gerar o módulo Python.
"""

import math
import os
import pty
import random
import socket
import subprocess
import sys
import tempfile
import time

ALVO = (1.20, 0.80)
BITOLA = 0.15
DIST_TRAS = 0.07


def conferir_checksum(linha):
    corpo = linha[linha.index("#") + 1:linha.rindex("*")]
    chk = 0
    for c in corpo:
        chk ^= ord(c)
    return chk == int(linha[linha.rindex("*") + 1:], 16), corpo


def carregar_proto_ssl():
    """Gera o messages_robocup_ssl_wrapper_pb2 a partir de proto/ numa pasta temporária."""
    pasta_proto = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "proto")
    saida = tempfile.mkdtemp(prefix="proto_ssl_")
    subprocess.run(["protoc", f"--proto_path={pasta_proto}", f"--python_out={saida}",
                    "messages_robocup_ssl_detection.proto",
                    "messages_robocup_ssl_geometry.proto",
                    "messages_robocup_ssl_wrapper.proto"], check=True)
    sys.path.insert(0, saida)
    import messages_robocup_ssl_wrapper_pb2
    return messages_robocup_ssl_wrapper_pb2


def pacote_ssl(pb, camera, quadro, robo, bola, extras):
    """Um frame de uma câmera, em mm. robo e bola podem ser None (fora da vista dela)."""
    p = pb.SSL_WrapperPacket()
    d = p.detection
    agora = time.time()
    d.frame_number = quadro
    d.camera_id = camera
    d.t_capture = agora - 0.004          # 4 ms de processamento na visão
    d.t_sent = agora
    for time_, rid, x, y, th, conf in ([("azul", 0, *robo, 0.9)] if robo else []) + extras:
        r = (d.robots_blue if time_ == "azul" else d.robots_yellow).add()
        r.confidence, r.robot_id, r.x, r.y, r.orientation = conf, rid, x, y, th
        r.pixel_x = r.pixel_y = 0
    if bola:
        b = d.balls.add()
        b.confidence, b.x, b.y, b.pixel_x, b.pixel_y = 0.9, bola[0], bola[1], 0, 0
    return p.SerializeToString()


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    exe = args[0] if args else "./build/ponte_serial"
    modo_ssl = "--ssl" in sys.argv
    mestre, escravo = pty.openpty()
    nome = os.ttyname(escravo)

    pb = carregar_proto_ssl() if modo_ssl else None
    entrada = ["--entrada", "ssl"] if modo_ssl else ["--porta-udp", "5005"]
    proc = subprocess.Popen(
        [exe, "--serial", nome, *entrada, "--hz", "200",
         "--hz-serial", "100", "--dist-parada", "0.10", "--silencioso"],
        stdout=subprocess.DEVNULL)
    time.sleep(2.5)  # a ponte espera 2 s por causa do reset do Arduino

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    if modo_ssl:
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 1)
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
    os.set_blocking(mestre, False)
    quadro = 0

    x, y, theta = 0.0, 0.0, math.radians(-40.0)
    t0 = time.time()
    dt = 1.0 / 60.0  # visão a 60 Hz
    ultimo_log = 0.0
    resto = b""

    print(f"alvo ({ALVO[0]:.2f}, {ALVO[1]:.2f}) | pose inicial (0.00, 0.00, -40 deg)\n")
    print("   t(s)   x      y     theta    d(m)   quadro serial")

    while time.time() - t0 < 12.0:
        # Ruído de câmera real (~1 mm, ~0,3 grau). Sem ele, pose idêntica em frames
        # seguidos com o robô parado seria tratada como visão travada.
        xr, yr = x + random.gauss(0, 0.001), y + random.gauss(0, 0.001)
        tr = theta + random.gauss(0, 0.005)
        if modo_ssl:
            quadro += 1
            robo = (xr * 1000, yr * 1000, tr)
            # câmera 0: robô, bola e uma deteção falsa do nosso id, fraca e longe
            sock.sendto(pacote_ssl(pb, 0, quadro, robo, (ALVO[0] * 1000, ALVO[1] * 1000),
                                   [("azul", 0, -3000.0, 2000.0, 0.0, 0.1)]),
                        ("224.5.23.2", 10020))
            # câmera 1: o mesmo robô com outra calibração (3 mm) e o amarelo 0, sem bola
            robo1 = (robo[0] + 3 + random.gauss(0, 1), robo[1] + random.gauss(0, 1), tr)
            sock.sendto(pacote_ssl(pb, 1, quadro, robo1, None,
                                   [("amarelo", 0, -1500.0, -800.0, 3.0, 0.95)]),
                        ("224.5.23.2", 10020))
        else:
            sock.sendto(f"pose {xr:.4f} {yr:.4f} {tr:.4f} {ALVO[0]:.4f} {ALVO[1]:.4f} "
                        f"{time.time():.3f}".encode(), ("127.0.0.1", 5005))
        time.sleep(dt)

        # Lê os quadros que a ponte escreveu e integra o movimento do "robô"
        try:
            resto += os.read(mestre, 4096)
        except BlockingIOError:
            pass
        linhas = resto.split(b"\n")
        resto = linhas[-1]
        ultima = None
        for b in linhas[:-1]:
            linha = b.decode(errors="ignore").strip()
            if "#" in linha and "*" in linha:
                ok, corpo = conferir_checksum(linha)
                if not ok:
                    print("CHECKSUM INVALIDO:", linha)
                    continue
                ultima = corpo

        if ultima:
            campos = ultima.split(",")
            ve, vd, vt = (int(c) / 1000.0 for c in campos[1:4])
            me, _md, mt = campos[4], campos[5], campos[6]
            if me == "A":
                v = 0.5 * (ve + vd)
                w = (vd - ve) / BITOLA
            elif mt == "A":
                v, w = 0.0, vt / DIST_TRAS
            else:
                v, w = 0.0, 0.0
            x += v * math.cos(theta) * dt
            y += v * math.sin(theta) * dt
            theta += w * dt

            t = time.time() - t0
            if t - ultimo_log > 0.5:
                ultimo_log = t
                d = math.hypot(ALVO[0] - x, ALVO[1] - y)
                print(f"{t:6.2f} {x:6.3f} {y:6.3f} {math.degrees(theta):7.1f} {d:6.3f}   "
                      f"#{ultima}")

    # Watchdog: para de mandar pose e confere se a ponte manda o robô parar
    print("\nvisao interrompida, conferindo a parada de segurança...")
    time.sleep(0.5)
    try:
        resto += os.read(mestre, 4096)
    except BlockingIOError:
        pass
    ultimas = [b.decode(errors="ignore").strip() for b in resto.split(b"\n") if b"#" in b]
    print("ultimo quadro:", ultimas[-1] if ultimas else "(nada)")

    proc.terminate()
    proc.wait(timeout=5)
    d = math.hypot(ALVO[0] - x, ALVO[1] - y)
    print(f"\ndistancia final ao alvo: {d:.3f} m (esperado ~0.10)")
    parou = bool(ultimas) and conferir_checksum(ultimas[-1])[1].split(",")[4:6] != ["A", "A"]
    if abs(d - 0.10) > 0.05 or not parou:
        print("FALHOU")
        sys.exit(1)


if __name__ == "__main__":
    main()
