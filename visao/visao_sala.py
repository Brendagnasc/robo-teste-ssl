"""
visao_sala.py

Câmera -> robô e bola em mm no campo -> SSL_WrapperPacket no multicast da liga.
Liga a visão (marcação de três círculos da OuroBots) na ponte_serial e na IA:

    câmera --> deteccao.py --> calibracao.py --> saida_ssl.py --multicast--> ponte_serial --entrada ssl
                (pixel)          (mm, rad)        224.5.23.2:10020           ssl-robot (ssl_vision)

Uso:

    python3 visao/calibracao.py --fonte 0 --saida visao/calibracao.json     # uma vez
    python3 visao/visao_sala.py --fonte 0 --calibracao visao/calibracao.json

    ./build/ponte_serial --serial /dev/ttyACM0 --entrada ssl --latencia 0.040

--fonte aceita também um arquivo de vídeo, tocado na velocidade dele (o teste usa isso).
--offset-frente-mm: distância do centro dos três círculos até o ponto que o controle usa
(o meio do eixo das rodas laterais), positiva se o ponto fica à frente do centro.
"""

import argparse
import json
import math
import os
import sys
import time

import cv2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from calibracao import Calibracao          # noqa: E402
from deteccao import CONFIG_PADRAO, detectar_bola, detectar_robo, preparar  # noqa: E402
from saida_ssl import GRUPO_PADRAO, PORTA_PADRAO, PublicadorSSL  # noqa: E402


def carregar_config(caminho):
    cfg = json.loads(json.dumps(CONFIG_PADRAO))   # cópia
    if caminho:
        with open(caminho) as f:
            cfg.update(json.load(f))
    return cfg


def desenhar(frame, robo_px, bola_px):
    if robo_px:
        u, v, ang, circulos = robo_px
        for cor, (x, y, r) in circulos.items():
            cv2.circle(frame, (int(x), int(y)), int(r), (255, 0, 255), 2)
            cv2.putText(frame, cor[0].upper(), (int(x) - 8, int(y) - int(r) - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
        ponta = (int(u + 50 * math.cos(ang)), int(v + 50 * math.sin(ang)))
        cv2.arrowedLine(frame, (int(u), int(v)), ponta, (0, 255, 255), 3, tipLength=0.2)
    if bola_px:
        cv2.circle(frame, (int(bola_px[0]), int(bola_px[1])), 6, (0, 140, 255), -1)


def main():
    ap = argparse.ArgumentParser(description="Visão da sala -> SSL_WrapperPacket")
    ap.add_argument("--fonte", default="0", help="câmera (0, 1, ...) ou arquivo de vídeo")
    ap.add_argument("--calibracao", default="visao/calibracao.json")
    ap.add_argument("--config", help="JSON com faixas HSV e parâmetros do Hough")
    ap.add_argument("--time", choices=["azul", "amarelo"], default="azul")
    ap.add_argument("--id", type=int, default=0)
    ap.add_argument("--offset-frente-mm", type=float, default=0.0)
    ap.add_argument("--grupo", default=GRUPO_PADRAO)
    ap.add_argument("--porta", type=int, default=PORTA_PADRAO)
    ap.add_argument("--interface", default="0.0.0.0")
    ap.add_argument("--sem-janela", action="store_true")
    args = ap.parse_args()

    cfg = carregar_config(args.config)
    cal = Calibracao.carregar(args.calibracao)
    pub = PublicadorSSL(args.time == "azul", args.id, grupo=args.grupo, porta=args.porta,
                        interface=args.interface)

    eh_camera = args.fonte.isdigit()
    cap = cv2.VideoCapture(int(args.fonte) if eh_camera else args.fonte)
    if not cap.isOpened():
        sys.exit(f"não abri a fonte {args.fonte}")
    # Arquivo: toca na taxa do vídeo, senão a ponte recebe tudo de uma vez
    periodo = 0.0 if eh_camera else 1.0 / (cap.get(cv2.CAP_PROP_FPS) or 30.0)

    print(f"visao: {args.fonte} -> SSL {args.grupo}:{args.porta}, robo {args.time} {args.id}")
    n = vistos = 0
    t_log = proximo = time.time()
    try:
        while True:
            ok, frame = cap.read()
            t_captura, m_captura = time.time(), time.monotonic()
            if not ok:
                break

            prep = preparar(frame, cfg)
            robo_px = detectar_robo(frame, cfg, prep)
            bola_px = detectar_bola(frame, cfg, prep)

            robo = bola = None
            pixel_robo = (0.0, 0.0)
            if robo_px:
                u, v, ang, _ = robo_px
                x, y = cal.pixel_para_campo(u, v)
                theta = cal.angulo_para_campo(u, v, ang)
                x += args.offset_frente_mm * math.cos(theta)
                y += args.offset_frente_mm * math.sin(theta)
                robo, pixel_robo = (x, y, theta), (u, v)
                vistos += 1
            if bola_px:
                bola = cal.pixel_para_campo(*bola_px)
            pub.enviar(t_captura, robo=robo, bola=bola, pixel_robo=pixel_robo,
                       pixel_bola=bola_px or (0.0, 0.0),
                       processamento_s=time.monotonic() - m_captura)
            n += 1

            if t_captura - t_log >= 1.0:
                pose = (f"({robo[0]:7.1f}, {robo[1]:7.1f}) mm {math.degrees(robo[2]):6.1f} graus"
                        if robo else "robo nao visto")
                alvo = f"bola ({bola[0]:7.1f}, {bola[1]:7.1f}) mm" if bola else "sem bola"
                print(f"{n / (t_captura - t_log):5.1f} fps  robo em {vistos}/{n}  {pose}  {alvo}")
                n = vistos = 0
                t_log = t_captura

            if not args.sem_janela:
                desenhar(frame, robo_px, bola_px)
                cv2.imshow("visao_sala", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            if periodo:
                proximo += periodo
                time.sleep(max(0.0, proximo - time.time()))
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        pub.fechar()
        if not args.sem_janela:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
