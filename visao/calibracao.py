"""
calibracao.py

Converte pixel da câmera em milímetros no chão, no referencial do campo, por homografia.

Por que homografia e não FOV + altura da câmera: com 4 ou mais pontos do chão medidos
com trena, ela resolve de uma vez a escala, a origem, o sentido dos eixos (a imagem tem
y para baixo, o campo tem y para a esquerda) e a perspectiva da câmera inclinada. Não
depende de FOV nem de altura, que são difíceis de medir direito.

Referencial do campo (o mesmo da ponte e do SSL-Vision): x para frente, y para a
esquerda, theta anti-horário a partir de +x, posição em mm. Escolha a origem no chão
(o centro da área de teste, por exemplo) e meça os pontos a partir dela.

Calibrar:

    # interativo: clique nos pontos na imagem e digite x y (mm) de cada um no terminal
    python3 visao/calibracao.py --fonte 0 --saida visao/calibracao.json

    # a partir de um arquivo com uma linha "u,v,x_mm,y_mm" por ponto
    python3 visao/calibracao.py --pontos pontos.csv --saida visao/calibracao.json

Use pontos espalhados pela área toda, de preferência nos cantos, e de 6 a 10 em vez de 4:
o erro de cada ponto aparece no fim e mostra se algum foi medido errado.

Depende só do numpy. O modo interativo precisa do OpenCV com janela (opencv-python).
"""

import argparse
import json
import math
import sys

import numpy as np


def _normalizar(pts):
    """Translada para a média e escala para distância média sqrt(2) (melhora a conta)."""
    media = pts.mean(axis=0)
    dist = np.sqrt(((pts - media) ** 2).sum(axis=1)).mean()
    s = math.sqrt(2) / dist if dist > 0 else 1.0
    T = np.array([[s, 0, -s * media[0]], [0, s, -s * media[1]], [0, 0, 1]])
    pts_h = np.column_stack([pts, np.ones(len(pts))])
    return (T @ pts_h.T).T[:, :2], T


def homografia(pixels, campo):
    """H tal que campo ~ H * pixel, por DLT normalizado. Precisa de 4 pontos ou mais."""
    pixels = np.asarray(pixels, dtype=float)
    campo = np.asarray(campo, dtype=float)
    if len(pixels) < 4 or len(pixels) != len(campo):
        raise ValueError("a homografia precisa de pelo menos 4 pares de pontos")
    p, Tp = _normalizar(pixels)
    c, Tc = _normalizar(campo)
    linhas = []
    for (u, v), (x, y) in zip(p, c):
        linhas.append([-u, -v, -1, 0, 0, 0, u * x, v * x, x])
        linhas.append([0, 0, 0, -u, -v, -1, u * y, v * y, y])
    _, _, vt = np.linalg.svd(np.asarray(linhas))
    Hn = vt[-1].reshape(3, 3)
    H = np.linalg.inv(Tc) @ Hn @ Tp
    return H / H[2, 2]


class Calibracao:
    def __init__(self, H):
        self.H = np.asarray(H, dtype=float)

    @classmethod
    def de_pontos(cls, pixels, campo):
        return cls(homografia(pixels, campo))

    @classmethod
    def carregar(cls, caminho):
        with open(caminho) as f:
            return cls(json.load(f)["homografia"])

    def salvar(self, caminho, pixels=None, campo=None):
        dados = {"homografia": self.H.tolist()}
        if pixels is not None:
            dados["pontos"] = [[float(u), float(v), float(x), float(y)]
                               for (u, v), (x, y) in zip(pixels, campo)]
            dados["erro_rms_mm"] = self.erro_rms(pixels, campo)
        with open(caminho, "w") as f:
            json.dump(dados, f, indent=2)

    def pixel_para_campo(self, u, v):
        """Pixel (float, sem arredondar) para (x, y) em mm no campo."""
        x, y, w = self.H @ np.array([u, v, 1.0])
        return x / w, y / w

    def angulo_para_campo(self, u, v, angulo_pixel, passo=20.0):
        """
        Direção medida na imagem (atan2 com y da imagem para baixo) para theta no campo.
        Converte dois pontos ao longo da direção, o que já leva em conta o y invertido
        e a perspectiva.
        """
        x0, y0 = self.pixel_para_campo(u, v)
        x1, y1 = self.pixel_para_campo(u + passo * math.cos(angulo_pixel),
                                       v + passo * math.sin(angulo_pixel))
        return math.atan2(y1 - y0, x1 - x0)

    def erros(self, pixels, campo):
        """Distância em mm entre cada ponto medido e onde a homografia o coloca."""
        return [math.hypot(*np.subtract(self.pixel_para_campo(u, v), (x, y)))
                for (u, v), (x, y) in zip(pixels, campo)]

    def erro_rms(self, pixels, campo):
        e = self.erros(pixels, campo)
        return math.sqrt(sum(d * d for d in e) / len(e))


# ------------------------------------------------------------------- calibrar

def ler_csv(caminho):
    pixels, campo = [], []
    with open(caminho) as f:
        for linha in f:
            linha = linha.strip()
            if not linha or linha.startswith("#"):
                continue
            u, v, x, y = (float(c) for c in linha.replace(";", ",").split(","))
            pixels.append((u, v))
            campo.append((x, y))
    return pixels, campo


def pontos_interativo(fonte):
    import cv2

    cap = cv2.VideoCapture(int(fonte) if str(fonte).isdigit() else fonte)
    ok, quadro = cap.read()
    cap.release()
    if not ok:
        sys.exit("não consegui ler a câmera")

    pixels, campo = [], []
    clique = []

    def ao_clicar(evento, u, v, *_):
        if evento == cv2.EVENT_LBUTTONDOWN:
            clique.append((u, v))

    cv2.namedWindow("calibracao")
    cv2.setMouseCallback("calibracao", ao_clicar)
    print("Clique num ponto do chão de posição conhecida e digite x y (mm) aqui.")
    print("Tecla q termina (precisa de 4 pontos ou mais).")
    while True:
        tela = quadro.copy()
        for i, (u, v) in enumerate(pixels):
            cv2.circle(tela, (int(u), int(v)), 5, (0, 0, 255), -1)
            cv2.putText(tela, str(i + 1), (int(u) + 6, int(v) - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.imshow("calibracao", tela)
        tecla = cv2.waitKey(30) & 0xFF
        if clique:
            u, v = clique.pop()
            texto = input(f"ponto {len(pixels) + 1} em ({u}, {v}) px -> x y em mm: ")
            try:
                x, y = (float(c) for c in texto.replace(",", " ").split())
            except ValueError:
                print("ponto ignorado: digite dois números")
                continue
            pixels.append((float(u), float(v)))
            campo.append((x, y))
        if tecla == ord("q") and len(pixels) >= 4:
            break
    cv2.destroyAllWindows()
    return pixels, campo


def main():
    ap = argparse.ArgumentParser(description="Calibração pixel -> mm por homografia")
    ap.add_argument("--fonte", help="câmera (0, 1, ...) ou arquivo de vídeo, modo interativo")
    ap.add_argument("--pontos", help="CSV com u,v,x_mm,y_mm por linha")
    ap.add_argument("--saida", default="visao/calibracao.json")
    args = ap.parse_args()

    if args.pontos:
        pixels, campo = ler_csv(args.pontos)
    elif args.fonte is not None:
        pixels, campo = pontos_interativo(args.fonte)
    else:
        ap.error("use --fonte ou --pontos")

    cal = Calibracao.de_pontos(pixels, campo)
    for i, e in enumerate(cal.erros(pixels, campo)):
        print(f"ponto {i + 1}: erro {e:6.1f} mm")
    print(f"erro RMS: {cal.erro_rms(pixels, campo):.1f} mm")
    cal.salvar(args.saida, pixels, campo)
    print(f"salvo em {args.saida}")


if __name__ == "__main__":
    main()
