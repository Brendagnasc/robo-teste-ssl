"""
deteccao.py

Acha o robô e a bola na imagem, em pixel, sem arredondar.

O robô usa a marcação do grupo da visão da OuroBots (repositório
VisaoComputacionalOuroBots): três círculos, frente azul (F), esquerda vermelha (E) e
direita preta (D). O centro do robô é o centro do triângulo e a frente é a direção do
centro até o F, como lá. O que muda é como os círculos são achados:

  - lá, HoughCircles na imagem cinza e depois a cor de cada círculo. No cinza, azul e
    vermelho ficam quase da cor de um chão médio, e o param2 = 80 só pega círculos
    grandes (raio perto de 100 px): no teste com círculos de 45 px nenhum foi achado.
    Aqui cada círculo sai direto da máscara da sua cor: contorno, área compatível com o
    raio esperado e circularidade;
  - o centro é o centróide da máscara, em float. Pose presa ao pixel (ou à grade do
    acumulador do Hough) se repete igual com o robô parado, e a ponte entende como
    visão travada;
  - o vermelho tem duas faixas (H perto de 0 e perto de 179). Só 160-179 perde o
    vermelho puro;
  - os três círculos precisam formar um triângulo quase equilátero, o que descarta
    sombra ou fita preta confundida com o círculo D;
  - a bola (laranja) também é detectada, pelo maior contorno da máscara.
"""

import math

import cv2
import numpy as np

# Faixas HSV: as do main.py da OuroBots, com a segunda faixa do vermelho. A bola é nossa.
# Precisam ser ajustadas com a luz e as cores do lugar (--config com um JSON).
CONFIG_PADRAO = {
    "frente": [[[90, 50, 80], [130, 255, 255]]],                    # azul
    "esquerda": [[[0, 100, 100], [6, 255, 255]],
                 [[160, 100, 100], [179, 255, 255]]],               # vermelho
    "direita": [[[0, 0, 0], [179, 255, 50]]],                       # preto
    "bola": [[[8, 120, 120], [22, 255, 255]]],                      # laranja
    "raio_min_px": 15,         # raio dos círculos da marcação na imagem
    "raio_max_px": 130,
    "circularidade_min": 0.6,  # 4*pi*area/perimetro^2: 1 no círculo perfeito
    "lados_razao_max": 1.5,    # maior lado / menor lado do triângulo F-E-D
    "bola_area_min_px": 40,
}

CORES = ("frente", "esquerda", "direita")


def mascara(hsv, faixas):
    m = None
    for minimo, maximo in faixas:
        parte = cv2.inRange(hsv, np.array(minimo, np.uint8), np.array(maximo, np.uint8))
        m = parte if m is None else cv2.bitwise_or(m, parte)
    return m


def preparar(frame_bgr, cfg):
    """Máscara de cada cor, com blur antes e abertura depois para tirar pontos soltos."""
    hsv = cv2.cvtColor(cv2.blur(frame_bgr, (4, 4)), cv2.COLOR_BGR2HSV)
    kernel = np.ones((3, 3), np.uint8)
    return {c: cv2.morphologyEx(mascara(hsv, cfg[c]), cv2.MORPH_OPEN, kernel)
            for c in CORES + ("bola",)}


def centroide(contorno):
    mom = cv2.moments(contorno)
    if mom["m00"] <= 0:
        return None
    return mom["m10"] / mom["m00"], mom["m01"] / mom["m00"], mom["m00"]


def achar_circulo(m, cfg):
    """O maior contorno da máscara com tamanho e forma de círculo: (x, y, raio) ou None."""
    area_min = math.pi * cfg["raio_min_px"] ** 2
    area_max = math.pi * cfg["raio_max_px"] ** 2
    melhor = None
    contornos, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for c in contornos:
        area = cv2.contourArea(c)
        if not area_min <= area <= area_max:
            continue
        perimetro = cv2.arcLength(c, True)
        if perimetro <= 0 or 4 * math.pi * area / perimetro ** 2 < cfg["circularidade_min"]:
            continue
        if melhor is None or area > melhor[1]:
            melhor = (c, area)
    if melhor is None:
        return None
    x, y, area = centroide(melhor[0])
    return x, y, math.sqrt(area / math.pi)


def detectar_robo(frame_bgr, cfg=CONFIG_PADRAO, preparado=None):
    """
    Devolve (u, v, angulo_pixel, circulos) ou None se faltar algum círculo ou se os três
    não formarem a marcação. angulo_pixel: atan2 do centro até a frente, y da imagem para baixo.
    """
    mascaras = preparado if preparado is not None else preparar(frame_bgr, cfg)
    circulos = {}
    for cor in CORES:
        c = achar_circulo(mascaras[cor], cfg)
        if c is None:
            return None
        circulos[cor] = c
    pts = [circulos[c][:2] for c in CORES]
    lados = [math.dist(pts[i], pts[(i + 1) % 3]) for i in range(3)]
    if min(lados) <= 0 or max(lados) / min(lados) > cfg["lados_razao_max"]:
        return None
    u = sum(p[0] for p in pts) / 3.0
    v = sum(p[1] for p in pts) / 3.0
    f = circulos["frente"]
    return u, v, math.atan2(f[1] - v, f[0] - u), circulos


def detectar_bola(frame_bgr, cfg=CONFIG_PADRAO, preparado=None):
    """Centro (u, v) em float do maior contorno laranja, ou None."""
    mascaras = preparado if preparado is not None else preparar(frame_bgr, cfg)
    contornos, _ = cv2.findContours(mascaras["bola"], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contornos:
        return None
    c = centroide(max(contornos, key=cv2.contourArea))
    if c is None or c[2] < cfg["bola_area_min_px"]:
        return None
    return c[0], c[1]
