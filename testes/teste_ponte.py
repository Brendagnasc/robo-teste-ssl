#!/usr/bin/env python3
"""
teste_ponte.py

Testa a ponte_serial sem robô e sem câmera: cria uma porta serial virtual (pty),
simula a visão mandando pose por UDP e mostra os quadros que chegariam ao Arduino.

    cmake -B build && cmake --build build
    python3 testes/teste_ponte.py ./build/ponte_serial

O simulador integra a velocidade comandada, então a pose "anda" e a FSM fecha o ciclo
inteiro: ALINHAMENTO, DESLOCAMENTO, FRENAGEM, PARADO.
"""

import math
import os
import pty
import socket
import subprocess
import sys
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


def main():
    exe = sys.argv[1] if len(sys.argv) > 1 else "./build/ponte_serial"
    mestre, escravo = pty.openpty()
    nome = os.ttyname(escravo)

    proc = subprocess.Popen(
        [exe, "--serial", nome, "--porta-udp", "5005", "--hz", "200",
         "--hz-serial", "100", "--dist-parada", "0.10", "--silencioso"],
        stdout=subprocess.DEVNULL)
    time.sleep(2.5)  # a ponte espera 2 s por causa do reset do Arduino

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    os.set_blocking(mestre, False)

    x, y, theta = 0.0, 0.0, math.radians(-40.0)
    t0 = time.time()
    dt = 1.0 / 60.0  # visão a 60 Hz
    ultimo_log = 0.0
    resto = b""

    print(f"alvo ({ALVO[0]:.2f}, {ALVO[1]:.2f}) | pose inicial (0.00, 0.00, -40 deg)\n")
    print("   t(s)   x      y     theta    d(m)   quadro serial")

    while time.time() - t0 < 12.0:
        sock.sendto(f"pose {x:.4f} {y:.4f} {theta:.4f} {ALVO[0]:.4f} {ALVO[1]:.4f} "
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


if __name__ == "__main__":
    main()
