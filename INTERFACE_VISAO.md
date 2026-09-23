# Interface visão -> IA/controle

Documento para acertar com o grupo da visão. Tudo o que a IA precisa está aqui, e
nada além disso. Se algum item mudar, o ajuste é só no `ConfigEntrada` do nosso lado,
mas precisamos saber qual é a convenção antes de rodar.

## 1. O que a visão entrega

Por frame, para cada robô detectado:

| campo | tipo | descrição |
|---|---|---|
| `id` | inteiro | identificador do robô (0 no robô de teste) |
| `x`, `y` | número | posição no plano do chão |
| `theta` | número | orientação, com a frente do robô apontando para `theta` |
| `t` | número | instante em que o FRAME FOI CAPTURADO, não o instante do envio |
| `confianca` | número (opcional) | 0 a 1, para descartarmos deteções fracas |

E, para a bola ou o objeto alvo: `x`, `y`, `t`, `confianca`.

## 2. Convenções a confirmar

Precisamos da resposta para cada item. Qualquer opção serve, desde que seja fixa.

- **Unidade de posição**: metros ou milímetros. O SSL-Vision usa milímetros.
- **Unidade de ângulo**: radianos ou graus. O SSL-Vision usa radianos.
- **Origem**: onde fica o `(0, 0)` da sala, fisicamente.
- **Sentido dos eixos**: nossa convenção é x para frente, y para a esquerda, theta
  anti-horário a partir de +x. Se a visão usa y invertido, avisem e nós espelhamos.
- **Zero do ângulo**: o marcador aponta para `theta = 0` em qual direção real.
- **Taxa de frames**: 30, 60 ou outra.
- **Latência total**: da exposição até o dado chegar na nossa porta. Mesmo uma
  estimativa grosseira ajuda muito (ver seção 4).

## 3. Transporte

Proposta: UDP em `127.0.0.1:5005`, uma linha de texto ASCII por frame:

```
pose <x> <y> <theta> <alvo_x> <alvo_y> <t>
```

É o formato mais simples de depurar, e um `netcat` já mostra o que está saindo.
Se preferirem protobuf ou JSON, funciona igual: a conversão fica do nosso lado, em um
único arquivo (`entrada_visao.cpp`). O que não pode mudar é o conjunto de campos.

Quando o robô ou a bola não for detectado no frame, **não mandem o último valor
conhecido**. Ou omitam o campo, ou mandem `confianca = 0`. Repetir a última posição
como se fosse nova nos impede de distinguir "parado" de "perdido", e nosso controle
precisa dessa diferença para acionar a parada de segurança.

## 4. O que fazemos com o dado, e por que a latência importa

O controle roda a 200 Hz e precisa da pose do instante atual. A visão entrega a pose
de alguns milissegundos atrás, a 30 ou 60 Hz. A camada `EntradaVisao` resolve isso:

1. converte unidades e referencial;
2. filtro alfa-beta estima posição e velocidade;
3. projeta a pose para o instante atual usando a latência informada;
4. preenche os ciclos sem frame novo;
5. descarta deteção errada (salto maior que o fisicamente possível).

Medição feita no nosso simulador, com visão a 60 Hz, 40 ms de atraso, 3 mm de ruído,
5% de frames perdidos e 1% de deteções erradas:

| | erro RMS da pose usada | pior erro | erro na parada |
|---|---|---|---|
| pose crua direto no controle | 124,4 mm | 1082,9 mm | 30,3 mm |
| com `EntradaVisao` | 13,1 mm | 57,5 mm | 18,7 mm |

A 3 m/s, cada 10 ms de atraso vale 3 cm de erro de posição. Por isso pedimos a
latência: é o parâmetro que mais muda a qualidade do resultado final, mais do que
a precisão do pixel.

## 5. Como medir a latência (10 minutos, juntos)

1. Deixe o robô parado e mande ele arrancar em linha reta com um comando conhecido.
2. Grave o instante do comando na ponte e o instante em que a visão acusa movimento.
3. A diferença é a latência total. Repita 5 vezes e use a média.

Alternativa sem robô: passe a mão na frente da câmera e compare o carimbo de tempo
do frame com o relógio do PC, desde que os dois relógios estejam sincronizados.

## 6. Nosso lado do contrato

Entregamos, na mesma taxa que a visão mandar:

- estado da FSM do robô (alinhando, deslocando, freando, parado);
- pose estimada e velocidade estimada;
- comandos enviados para cada roda.

Isso ajuda vocês a validarem a visão: se a nossa estimativa e a de vocês divergem
com o robô parado, o problema é ruído; se divergem só com o robô rápido, é latência.
