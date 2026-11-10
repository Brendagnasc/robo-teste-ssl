# Teste com o robô real

Roteiro do primeiro teste completo: câmera, visão da sala, ponte e robô.

```
câmera --> visao_sala.py --multicast--> ponte_serial --entrada ssl --serial--> firmware_robo
```

Ainda não foi feito com câmera e robô de verdade. Anote o que der diferente.

## O que o sistema faz hoje

- "Mapa" é a calibração da câmera: ela converte pixel em milímetros no chão. Não existe
  mapa de obstáculos;
- o caminho é uma reta até o alvo. O robô gira com a roda traseira até apontar para ele
  e depois anda, sem desviar de nada;
- velocidade e aceleração saem do controle: aceleração constante e frenagem calculadas
  pela aderência, pelo tombamento e pelo motor. Só valem com os parâmetros do robô real
  (passo 2);
- o alvo é a bola laranja que a visão detecta. Ponha a bola, ou um círculo laranja, onde
  o robô tem que chegar.

## 0. Preparar o PC (uma vez)

```bash
cmake -B build && cmake --build build
python3 -m venv .venv && .venv/bin/pip install -r visao/requirements.txt
```

No WSL:

- webcam USB costuma não funcionar, mesmo anexada com o `usbipd`. Se não aparecer um
  `/dev/video0`, rode em Linux nativo;
- o WSL não enxerga o Bluetooth da máquina. Com o firmware em `USAR_BLUETOOTH 1`, os
  comandos pela USB são ignorados. No WSL, use `USAR_BLUETOOTH 0` e o cabo, anexado com
  o `usbipd`.

## 1. Firmware: medir e gravar

Com o robô SUSPENSO, rodas no ar, confira o sentido de cada roda. Se alguma girar ao
contrário, troque o `SENTIDO` dela no `.ino`.

```bash
python3 testes/robo_bancada.py /dev/ttyACM0 vel 300 300 0 --duracao 2
python3 testes/robo_bancada.py /dev/ttyACM0 vel 0 0 200 --modos I I A --duracao 2
```

No chão, meça as duas constantes do topo do `.ino`:

```bash
python3 testes/robo_bancada.py /dev/ttyACM0 deadband          # PWM_MINIMO
python3 testes/robo_bancada.py /dev/ttyACM0 vmax --duracao 2  # V_MAX_MM_S, com trena
python3 testes/robo_bancada.py /dev/ttyACM0 watchdog          # rodas param em até 200 ms
```

Grave o firmware com os valores medidos. Com Bluetooth, troque a porta por `/dev/rfcomm0`.

## 2. Parâmetros do robô na IA

A aceleração e a frenagem saem do `ParametrosFisicos`, em `include/ssl_controle.hpp`,
que hoje tem os números de um robô SSL de exemplo. Troque pelos do robô: massa, raio
da roda, bitola, distância até a roda traseira, posição e altura do centro de massa,
torque, rpm e redução do motor. Depois recompile.

Se o centro de massa não ficar entre o eixo lateral e a roda traseira, a ponte se
recusa a andar: sem carga na traseira, ela não consegue girar o robô.

Para a simulação continuar parecida com o robô, mude os mesmos números em
`simulacao/robo_teste.sdf`.

## 3. Marcação no robô

Três círculos por cima: azul na frente, vermelho à esquerda, preto à direita. Meça em
mm a distância entre o centro do triângulo e o meio do eixo das rodas laterais. Esse
valor vai no `--offset-frente-mm`.

## 4. Câmera e calibração

1. Prenda a câmera no alto, vendo o chão inteiro, e não mexa mais nela.
2. Marque de 6 a 10 pontos no chão com fita e meça o x e o y de cada um, em mm, a partir
   de uma origem escolhida.
3. Rode a calibração, clique em cada ponto na imagem e digite o x e o y dele:

```bash
.venv/bin/python visao/calibracao.py --fonte 0 --saida visao/calibracao.json
```

A pose do robô e a do alvo passam a sair nessas coordenadas. Câmera mexida, calibração
de novo.

## 5. Testar a visão sozinha

```bash
.venv/bin/python visao/visao_sala.py --fonte 0 --calibracao visao/calibracao.json --offset-frente-mm 0
```

Troque o 0 pelo valor do passo 3. Na janela, confira se o robô e a bola aparecem, com o
robô parado e movido na mão. Se uma cor falhar, ajuste as faixas HSV em
`deteccao.CONFIG_PADRAO` ou num JSON passado com `--config`: dependem da luz do lugar.

## 6. Rodar tudo

Com a visão rodando, em outro terminal:

```bash
./build/ponte_serial --serial /dev/ttyACM0 --entrada ssl --latencia 0.030 --v-roda-max 0.9 --dist-parada 0.11
```

- `--v-roda-max`: o `V_MAX_MM_S` medido, em m/s;
- `--dist-parada 0.11`: para encostado na bola (raio do robô + raio da bola). Com um
  círculo no chão, onde o robô tem que parar em cima, use `0`;
- `--latencia`: o atraso que o pacote da visão não mede, exposição da câmera e rede. Como
  medir: seção 5 do `INTERFACE_VISAO.md`.

Ponha a bola no ponto: o robô alinha, acelera, freia e para. Mude a bola de lugar e ele
vai atrás. A ponte mostra no log, a cada 0,5 s, o estado, a distância, o erro de ângulo,
as velocidades e os frames da visão aceitos, descartados e repetidos.

Na primeira vez, fique com a mão perto. O robô para sozinho quando a visão some por
100 ms, quando ela trava, quando ele não se move como foi comandado e quando a serial
cai por 200 ms (watchdog). Neste último caso os motores ficam em inércia e ele ainda
desliza.

## Se algo der errado

Guarde umas 10 linhas do log da ponte, o que o robô fez e, se a deteção falhar, um print
da janela da visão.

| sintoma | causa provável |
|---|---|
| `[VISAO TRAVADA]` ou muitos repetidos | a visão manda pose idêntica: o robô aparece na janela? |
| `[SEM MOVIMENTO: parado]` | roda patinando, motor invertido (`SENTIDO`) ou firmware ouvindo outro canal |
| gira para o lado errado | marcação trocada (azul na frente) ou eixo y invertido na calibração |
| para longe do ponto ou passa dele | `--offset-frente-mm`, `--latencia` ou `V_MAX_MM_S` diferentes do real |
| frenagem não move o robô | `PWM_MINIMO` baixo demais |
| para 1 a 2 cm antes | normal: é a tolerância `tol_posicao_m` de 2 cm |
