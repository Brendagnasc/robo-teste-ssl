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
                                         |
                            EntradaVisao + ControladorSSL
```

- visão: 30 ou 60 Hz, com latência;
- controle: 200 Hz;
- serial: 100 Hz, quadro de texto com checksum XOR.

## Módulos

| arquivo | papel |
|---|---|
| `include/ssl_controle.hpp`, `src/ssl_controle.cpp` | física (inércia, aderência, tombamento), rampas de aceleração constante e máquina de estados |
| `include/entrada_visao.hpp`, `src/entrada_visao.cpp` | conversão de unidades, filtro alfa-beta, compensação de latência, rejeição de deteção errada |
| `src/ponte_serial.cpp` | recebe UDP, roda o controle a 200 Hz, envia os quadros pela serial |
| `src/main_simulacao.cpp` | planta simulada + controle, sem hardware |
| `src/teste_entrada_visao.cpp` | mede o ganho da camada de entrada contra a pose crua |
| `firmware/firmware_robo/` | sketch Arduino: interpreta os quadros, aciona as pontes H, watchdog |
| `testes/teste_ponte.py` | teste de ponta a ponta com porta serial virtual (pty) |

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
python3 testes/teste_ponte.py           # ponta a ponta, com serial virtual

# com o robô conectado
./build/ponte_serial --serial /dev/ttyACM0 --milimetros --latencia 0.045
```

No WSL, `/dev/ttyACM0` só aparece depois de anexar o dispositivo com `usbipd-win`.

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

- sem frame da visão por 150 ms: a pose vence e o controle freia com rampa;
- sem quadro serial por 200 ms: o watchdog do firmware coloca tudo em inércia;
- salto impossível na visão: descartado, e aceito se repetir três vezes seguidas.

## Estado atual

Feito: física e limites, FSM, camada de entrada da visão, ponte serial, firmware em
malha aberta, testes de simulação e de ponta a ponta.

Próximos passos: PID de velocidade por roda quando confirmarmos os encoders, camada de
decisão acima da FSM (escolha de alvo) e desvio de obstáculos.
