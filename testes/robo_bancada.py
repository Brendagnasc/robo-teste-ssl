#!/usr/bin/env python3
"""
robo_bancada.py

Fala direto com o firmware pela serial, sem visão e sem a ponte. É a ferramenta dos
primeiros testes com o robô real: conferir sentido de rotação, medir V_MAX_MM_S,
achar o PWM_MINIMO e testar o watchdog.

Requer pyserial:  pip install pyserial

Comandos:

  # roda a 300 mm/s por 2 s, robô SUSPENSO, para conferir o sentido de cada roda
  python3 testes/robo_bancada.py /dev/ttyACM0 vel 300 300 0 --duracao 2

  # giro no próprio eixo (só a roda traseira, laterais em inércia)
  python3 testes/robo_bancada.py /dev/ttyACM0 vel 0 0 200 --modos I I A --duracao 2

  # PWM direto, para calibração (não passa pela conversão de velocidade)
  python3 testes/robo_bancada.py /dev/ttyACM0 pwm 150 150 0 --duracao 2

  # rampa de PWM: sobe de 10 em 10 e você anota onde a roda começa a girar
  python3 testes/robo_bancada.py /dev/ttyACM0 deadband

  # mede V_MAX: PWM 255 por um tempo fixo, você mede a distância com trena
  python3 testes/robo_bancada.py /dev/ttyACM0 vmax --duracao 2

  # para de enviar e confere se o watchdog do firmware corta os motores
  python3 testes/robo_bancada.py /dev/ttyACM0 watchdog

Segurança: os três primeiros testes devem ser feitos com o robô SUSPENSO, rodas no ar.
Só desça no chão depois de confirmar o sentido de rotação de cada roda.
"""

import argparse
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("Falta o pyserial. Instale com: pip install pyserial")


def quadro(corpo: str) -> bytes:
    """Monta o quadro com o mesmo checksum XOR que a ponte usa."""
    chk = 0
    for c in corpo:
        chk ^= ord(c)
    return f"#{corpo}*{chk:02X}\n".encode("ascii")


def enviar(ser, corpo, duracao, hz=50):
    """Repete o quadro durante 'duracao'. Precisa repetir: o watchdog corta em 200 ms."""
    fim = time.time() + duracao
    while time.time() < fim:
        ser.write(quadro(corpo))
        time.sleep(1.0 / hz)


def parar(ser):
    enviar(ser, "V,0,0,0,I,I,I", 0.3)


def main():
    ap = argparse.ArgumentParser(description="Teste de bancada do robô pela serial")
    ap.add_argument("porta", help="ex.: /dev/ttyACM0")
    ap.add_argument("comando", choices=["vel", "pwm", "deadband", "vmax", "watchdog"])
    ap.add_argument("valores", nargs="*", type=int, help="esq dir tras")
    ap.add_argument("--modos", nargs=3, default=["A", "A", "I"],
                    help="modo de cada roda: A ativo, I inércia, F freio")
    ap.add_argument("--duracao", type=float, default=2.0)
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    ser = serial.Serial(args.porta, args.baud, timeout=0.1)
    time.sleep(2.0)   # o Arduino reinicia quando a porta abre

    try:
        if args.comando == "vel":
            e, d, t = (args.valores + [0, 0, 0])[:3]
            me, md, mt = args.modos
            print(f"esq {e} mm/s ({me}), dir {d} mm/s ({md}), tras {t} mm/s ({mt}), "
                  f"por {args.duracao:.1f} s")
            enviar(ser, f"V,{e},{d},{t},{me},{md},{mt}", args.duracao)

        elif args.comando == "pwm":
            e, d, t = (args.valores + [0, 0, 0])[:3]
            print(f"PWM direto: {e}, {d}, {t} por {args.duracao:.1f} s")
            enviar(ser, f"P,{e},{d},{t}", args.duracao)

        elif args.comando == "deadband":
            print("Rampa de PWM nas rodas laterais, robô NO CHÃO.")
            print("Anote o valor em que as rodas começam a girar: esse é o PWM_MINIMO.\n")
            for pwm in range(0, 161, 10):
                print(f"  PWM = {pwm}", flush=True)
                enviar(ser, f"P,{pwm},{pwm},0", 1.5)
            parar(ser)

        elif args.comando == "vmax":
            print(f"PWM 255 nas laterais por {args.duracao:.1f} s.")
            print("Meça a distância percorrida com trena. V_MAX_MM_S = distancia_mm / tempo_s")
            input("Alinhe o robô e pressione Enter para começar...")
            for i in (3, 2, 1):
                print(i, flush=True)
                time.sleep(1)
            t0 = time.time()
            enviar(ser, "P,255,255,0", args.duracao)
            parar(ser)
            print(f"tempo real de acionamento: {time.time() - t0:.2f} s")

        elif args.comando == "watchdog":
            print("Acionando por 2 s e depois PARANDO de enviar.")
            print("As rodas têm que parar em até 200 ms sozinhas.")
            enviar(ser, "V,200,200,0,A,A,I", 2.0)
            print("envio interrompido agora")
            time.sleep(2.0)

    finally:
        parar(ser)
        ser.close()
        print("motores em inércia, porta fechada")


if __name__ == "__main__":
    main()
