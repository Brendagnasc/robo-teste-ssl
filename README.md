# robo-teste-ssl

Camada de IA e controle do robô de teste físico, precursora do sistema da RoboCup SSL.

Recebe a pose vinda da visão computacional, decide o que fazer e comanda os motores
pela USB serial. A visão é feita por outro grupo; o contrato entre as duas partes está
em [INTERFACE_VISAO.md](INTERFACE_VISAO.md).

## Arquitetura do robô

Três rodas, com acionamento intermitente:

- duas rodas laterais convencionais, para translação;
- uma roda traseira omni, montada tangencialmente, exclusiva para giro;
- quando um grupo está ativo, o outro fica em inércia (motor livre).

O centro de massa precisa ficar entre o eixo das rodas laterais e a roda traseira,
senão a traseira não tem carga normal e não consegue girar o robô. O código valida
isso na inicialização e se recusa a mover com geometria inválida.

## Fluxo em execução

```
visão (outro grupo)  --UDP 5005-->  ponte_serial  --USB serial-->  firmware_robo
SSL-Vision / grSim   --multicast-->       |
                     224.5.23.2:10020     |
                            EntradaVisao + ControladorSSL
```

A ponte aceita duas entradas: a linha de texto `pose ...` do contrato com o outro grupo
(padrão) ou, com `--entrada ssl`, o `SSL_WrapperPacket` que o SSL-Vision, o grSim e o
ER-Force mandam. Ver [Entrada no formato da liga](#entrada-no-formato-da-liga).

- visão: 30 ou 60 Hz, com latência;
- controle: 200 Hz;
- serial: 100 Hz, quadro de texto com checksum XOR.

## Módulos

| arquivo | papel |
|---|---|
| `include/ssl_controle.hpp`, `src/ssl_controle.cpp` | física (inércia, aderência, tombamento), rampas de aceleração constante e máquina de estados |
| `include/entrada_visao.hpp`, `src/entrada_visao.cpp` | conversão de unidades, filtro alfa-beta, compensação de latência, rejeição de deteção errada |
| `src/ponte_serial.cpp` | recebe UDP, roda o controle a 200 Hz, envia os quadros pela serial |
| `include/visao_ssl.hpp`, `src/visao_ssl.cpp` | lê o `SSL_WrapperPacket` (time, id, confiança, tempo de processamento) e abre o multicast |
| `proto/` | `.proto` da visão copiados do SSL-Vision |
| `src/main_simulacao.cpp` | planta simulada + controle, sem hardware |
| `src/teste_entrada_visao.cpp` | mede o ganho da camada de entrada contra a pose crua |
| `src/teste_seguranca.cpp` | visão travada, robô preso, motor invertido: confere que o robô para |
| `src/teste_visao_ssl.cpp` | leitura dos pacotes SSL e ganho da latência por frame |
| `firmware/firmware_robo/` | sketch Arduino: interpreta os quadros, aciona as pontes H, watchdog |
| `testes/teste_ponte.py` | teste de ponta a ponta com porta serial virtual (pty) |
| `simulacao/robo_teste.sdf` | mundo e robô do Gazebo, com os números do `ParametrosFisicos` |
| `simulacao/robo_simulado.cpp` | plugin do Gazebo no lugar do hardware: firmware, motores A/I/F e câmera |
| `simulacao/rodar_gazebo.sh` | sobe o Gazebo e liga nele a `ponte_serial` de verdade |
| `visao/visao_sala.py` | câmera -> robô e bola em mm -> SSL no multicast (ver [Visão da sala](#visão-da-sala)) |
| `visao/deteccao.py` | acha os três círculos da marcação e a bola, em float |
| `visao/calibracao.py` | pixel -> mm no chão por homografia, e a ferramenta para calibrar |
| `visao/saida_ssl.py` | monta e manda o `SSL_WrapperPacket`, sem depender do protobuf |
| `testes/teste_visao_sala.py` | visão com imagens sintéticas de pose conhecida, até o multicast |

## Máquina de estados

```
PARADO --(alvo longe)--> ALINHAMENTO --(erro < 2 graus)--> DESLOCAMENTO
   ^                                                            |
   |                                    (distância <= distância de frenagem)
   |                                                            v
   +-------------(v = 0, no alvo)---------------------------FRENAGEM
                                                                |
 ALINHAMENTO <-------(v = 0, erro de rumo > 15 graus)-----------+
```

O perfil de velocidade é trapezoidal: aceleração constante até a velocidade máxima e
frenagem por `v = sqrt(2*a*d)`, virando linear perto do alvo para chegar sem oscilar.
Os limites de aceleração não são chutados: saem da aderência com transferência de
carga, do limite de tombamento e do torque dos motores.

## Como rodar

```bash
cmake -B build && cmake --build build

./build/simulacao                       # controle contra planta simulada
./build/teste_entrada                   # ganho da camada de entrada
./build/teste_seguranca                 # proteções contra o robô fugir (sai com 1 se falhar)
./build/teste_visao_ssl                 # leitura do protobuf da liga (sai com 1 se falhar)
python3 testes/teste_ponte.py           # ponta a ponta, com serial virtual
python3 testes/teste_ponte.py --ssl     # idem, com a visão em protobuf no multicast
.venv/bin/python testes/teste_visao_sala.py   # visão com imagens sintéticas (ver Visão da sala)
./simulacao/rodar_gazebo.sh             # ponte_serial contra o robô no Gazebo (ver abaixo)

# com o robô por cabo
./build/ponte_serial --serial /dev/ttyACM0 --milimetros --latencia 0.045

# com o robô por Bluetooth (HC-05)
./scripts/bluetooth_conectar.sh 98:D3:31:XX:XX:XX
./build/ponte_serial --serial /dev/rfcomm0 --bluetooth --milimetros --latencia 0.045

# com a visão no formato da liga (SSL-Vision, grSim)
./build/ponte_serial --serial /dev/ttyACM0 --entrada ssl --time azul --id 0
```

No WSL, `/dev/ttyACM0` só aparece depois de anexar o dispositivo com `usbipd-win`.

## Entrada no formato da liga

Com `--entrada ssl` a ponte entra no grupo `224.5.23.2:10020` e lê o `SSL_WrapperPacket`.
Assim o mesmo controle roda com o SSL-Vision e uma câmera de verdade, com o grSim ou
com a visão do outro grupo, se ela publicar nesse formato. É também o que a IA do
`ssl-robot` já consome, então visão, IA e robô passam a falar a mesma coisa.

- o robô seguido é o `--id` do `--time` (azul ou amarelo); o alvo é a bola;
- posição em mm, convertida sozinha. A origem é o centro do campo;
- cada câmera manda o seu pacote. A ponte fica com a deteção mais recente do robô e
  da bola, mesmo que venham de câmeras diferentes. Sem bola no frame, o alvo continua
  sendo a última posição vista;
- deteção abaixo de `--confianca-min` (0,3) é ignorada; com o mesmo id duas vezes no
  frame, fica a mais confiável;
- latência de cada frame: o `--latencia` mais `t_sent - t_capture` do pacote. Essa
  diferença é o processamento da visão, medido no relógio dela, então vale mesmo com os
  PCs fora de sincronia. Nesse modo, `--latencia` é só o que o pacote não vê: exposição
  da câmera e rede. Com o atraso variando de 15 a 65 ms, isso tira o erro da pose de
  19 mm para 5 mm (`teste_visao_ssl`);
- `--grupo`, `--porta-udp` e `--interface` trocam o endereço e a placa de rede, para
  quando a visão roda em outro PC.

Precisa do `libprotobuf-dev`; sem ele o `cmake` compila a ponte só com a entrada de texto.

No grSim, ligue `Gaussian noise` nas configurações. Sem ruído, o robô parado sai com a
pose idêntica em todo frame, a ponte entende como visão travada e não deixa ele andar.
Com o ruído ligado, a ponte acompanha o robô e a bola certinho (conferido movendo a bola
pelo grSim). O robô do grSim não anda, porque a ponte manda os comandos pela serial e não
para ele, então o log mostra `[SEM MOVIMENTO: parado]`, como esperado.

## Visão da sala

`visao/` liga uma câmera comum à ponte, no formato da liga. Usa a marcação do grupo da
visão da OuroBots (repositório `VisaoComputacionalOuroBots`): três círculos, frente azul,
esquerda vermelha e direita preta. O centro do robô é o centro do triângulo e a frente
aponta para o círculo azul.

```
câmera --> deteccao.py --> calibracao.py --> saida_ssl.py --multicast--> ponte_serial --entrada ssl
             (pixel)        (mm, rad)                                    ssl-robot (IA)
```

```bash
python3 -m venv .venv && .venv/bin/pip install -r visao/requirements.txt

# uma vez, com a câmera no lugar: clique em 6 a 10 pontos do chão e digite x y (mm) de cada
.venv/bin/python visao/calibracao.py --fonte 0 --saida visao/calibracao.json

.venv/bin/python visao/visao_sala.py --fonte 0 --calibracao visao/calibracao.json
./build/ponte_serial --serial /dev/ttyACM0 --entrada ssl --latencia 0.030
```

Em relação ao código da OuroBots, mudou:

- a conversão para mm é por homografia com pontos medidos no chão. A fórmula com FOV e
  distância focal tinha o FOV em graus dentro do `tan` (distância 32% menor), o mesmo FOV
  nos dois eixos e a origem no canto da imagem com y para baixo;
- cada círculo sai da máscara da sua cor (contorno, tamanho, circularidade) e não do
  `HoughCircles` no cinza. No cinza, azul e vermelho ficam quase da cor do chão, e o
  `param2 = 80` só pega círculos grandes: no teste, com círculos de 45 px, não achava nada;
- centro e ângulo em float, pelo centróide da cor. Em pixel inteiro a pose do robô
  parado se repete e a ponte trava achando que a visão travou;
- vermelho com as duas faixas de H (perto de 0 e de 179); os três círculos precisam
  formar um triângulo quase equilátero; a bola laranja é detectada;
- quando o robô não aparece, o pacote vai sem ele, sem previsão no lugar.

As faixas HSV e os tamanhos ficam em `deteccao.CONFIG_PADRAO` e podem vir de um JSON com
`--config`. Precisam ser ajustados com a luz e as cores do lugar. `--offset-frente-mm`
corrige a distância entre o centro da marcação e o meio do eixo das rodas, que é o ponto
que o controle usa.

O `saida_ssl.py` funciona sozinho (só biblioteca padrão), então o grupo da visão pode
copiar e chamar `PublicadorSSL().enviar(t_captura, robo=(x, y, theta), bola=(x, y))` no
código deles.

Medido com imagens sintéticas (câmera inclinada, ruído, chão cinza e carpete verde): robô
com erro de 4,5 mm e 0,6 grau, bola com 2 mm, nenhum quadro perdido. Contra a
`ponte_serial`, robô parado e bola a 0,652 m: a ponte aceitou os 240 quadros, sem nenhum
repetido, e calculou d = 0,652 m. Ainda não foi testado com câmera de verdade.

## Simulação no Gazebo

O plugin `robo_simulado` faz o papel do hardware, então a `ponte_serial` roda contra o
Gazebo sem nenhuma mudança, pela mesma serial e pelo mesmo UDP do robô real:

```
ponte_serial --serial /tmp/robo_teste_serial --pty--> firmware simulado --torque--> rodas
     ^                                                                               |
     +---------------- UDP 5005 "pose ..." (60 Hz, 40 ms, 2 mm) <---- câmera simulada <-+
```

```bash
./simulacao/rodar_gazebo.sh                # com janela: arraste a bola (tecla T) e o robô vai atrás
./simulacao/rodar_gazebo.sh --sem-janela   # só o servidor, mais leve
./simulacao/rodar_gazebo.sh --sem-ponte    # só o robô, para comandar com o testes/robo_bancada.py
```

Precisa do Gazebo Harmonic; sem ele, o `cmake` só pula o plugin.

- firmware: quadros `#V` e `#P`, checksum, velocidade zero virando inércia e watchdog,
  como no `firmware_robo.ino`;
- motores: torque calculado a cada passo de 1 ms, saturado em 0,32 N·m. Inércia é torque
  zero, com a roda livre de verdade, o que o `JointController` do Gazebo não oferece;
- câmera: taxa, atraso, ruído, perda de frame e deteção errada ajustáveis no `.sdf`.

Medido sem janela, com a bola em (1,0; 0,5): o robô alinha 25 graus em 0,4 s, acelera a
~2,9 m/s², anda a 0,89 m/s e para a 0,12 m da bola (alvo: 0,11). Com a bola atrás dele,
gira 133 graus em cerca de 1 s só com a traseira (até 5 rad/s), as laterais livres.

Cuidados:

- o controle calcula `v_max` de 3,0 m/s pelo `ParametrosFisicos`, mas o firmware satura
  em `V_MAX_MM_S` (900 mm/s). No simulador a ponte chega a pedir 1,9 m/s e o robô fica em
  0,89. Depois de medir o `V_MAX_MM_S` real, limite o controle a ele (`v_max_estrategia`);
- em inércia o robô desliza bastante, porque o único atrito é o `<friction>` de
  0,001 N·m das juntas. Mais atrito encurta a deslizada, mas também freia o giro: as
  laterais giram livres quando a traseira vira o robô. Meça no robô real quanto ele
  desliza com os motores cortados;
- `ganho_Nms` acima de 0,036 faz o laço da roda divergir e o Gazebo abortar. Era o que
  fechava a simulação com o `p_gain` 0,2 dos `JointController`;
- no WSL com duas placas de vídeo, o OpenGL pode cair na integrada:
  `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA` força a NVIDIA.

## Canal de comando: cabo ou Bluetooth

O protocolo é o mesmo nos dois casos, então trocar de canal não muda o controle.

| | cabo USB | Bluetooth (HC-05) |
|---|---|---|
| porta no PC | `/dev/ttyACM0` | `/dev/rfcomm0` |
| velocidade | 115200 bps | 38400 bps |
| envio de quadros | 100 Hz | 50 Hz (`--bluetooth`) |
| watchdog do firmware | 200 ms | 300 ms |
| pinos no Arduino | 0 e 1 (USB) | 10 (RX) e 9 (TX), via `SoftwareSerial` |

Escolha o canal em `USAR_BLUETOOTH`, no topo do `.ino`. O HC-05 não vai nos pinos 0 e 1,
que são os do USB e impediriam o upload: fica em `SoftwareSerial`, deixando a USB livre
para gravar e depurar. O RXD do módulo é 3,3 V e precisa de divisor resistivo (1k em
série, 2k para o GND) no TX do Arduino.

O WSL não enxerga o Bluetooth interno da máquina, e o `usbipd` não resolve isso. Para
usar Bluetooth, rode a ponte em Linux nativo ou use um dongle Bluetooth USB anexado
ao WSL.

## Resultados medidos

Visão a 60 Hz, 40 ms de latência, 3 mm de ruído, 5% de frames perdidos, 1% de deteção
errada, média de 20 corridas:

| | erro RMS da pose usada | pior erro | erro na parada |
|---|---|---|---|
| pose crua direto no controle | 124,4 mm | 1082,9 mm | 30,3 mm |
| com `EntradaVisao` | 13,1 mm | 57,5 mm | 18,7 mm |

## Antes de ligar o robô

Duas medidas precisam ir para o firmware (constantes no topo do `.ino`):

- `V_MAX_MM_S`: velocidade da roda em PWM 100%, medida com trena e cronômetro;
- `PWM_MINIMO`: PWM em que a roda começa a girar com o robô no chão.

Sem compensar a zona morta, os comandos lentos da frenagem não movem o robô.

## Segurança

- sem frame da visão por 100 ms: o controle freia com rampa;
- visão travada (pose idêntica repetida): frame repetido conta como perdido, e com
  6 seguidos (~100 ms) o controle freia na hora. Sem isso, a pose parada parece
  "robô longe do alvo" e o controle acelera até a velocidade máxima;
- comandou movimento e a visão não mostrou: se em 300 ms a visão não mostra pelo
  menos 25% do deslocamento ou giro comandado, no sentido certo, o robô para e espera
  1 s. Pega robô preso, roda patinando e motor ligado ao contrário;
- sem quadro serial por 200 ms: o watchdog do firmware coloca tudo em inércia;
- salto impossível na visão: descartado, e aceito se repetir três vezes seguidas.

## Estado atual

Feito: física e limites, FSM, camada de entrada da visão, ponte serial, firmware em
malha aberta, testes de simulação e de ponta a ponta, o robô no Gazebo comandado pela
ponte de verdade, a entrada no formato da liga (SSL-Vision/grSim) e a visão da sala
publicando nesse formato.

Próximos passos: PID de velocidade por roda quando confirmarmos os encoders, camada de
decisão acima da FSM (escolha de alvo) e desvio de obstáculos.
