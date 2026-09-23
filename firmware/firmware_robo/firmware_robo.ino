/**
 * firmware_robo.ino
 *
 * Firmware do robô de teste: recebe as velocidades das rodas pela USB serial e
 * aciona as três pontes H. Funciona em malha aberta (sem encoder) e já está preparado
 * para malha fechada quando você confirmar se os motores têm encoder.
 *
 * Quadros recebidos:
 *   operação (gerado pela ponte_serial):
 *     #V,<esq_mm_s>,<dir_mm_s>,<tras_mm_s>,<me>,<md>,<mt>*<checksum_hex>\n
 *     modos: A = ativo (segue velocidade), I = inércia (coast, motor livre),
 *            F = freio (curto entre os terminais, segura o robô parado)
 *   calibração (PWM direto, sem passar pela conversão de velocidade):
 *     #P,<pwm_esq>,<pwm_dir>,<pwm_tras>*<checksum_hex>\n
 *     valores de -255 a 255. Serve para medir V_MAX_MM_S e PWM_MINIMO.
 *
 * Segurança (watchdog): se passar TIMEOUT_MS sem um quadro válido, tudo vai para
 * inércia. É o que evita o robô sair correndo se o cabo cair ou a ponte travar.
 *
 * Ligação assumida: driver tipo TB6612FNG ou L298N, com IN1/IN2 de direção e PWM
 * de velocidade por motor. Ajuste os pinos abaixo conforme a sua placa.
 */

#include <Arduino.h>

// ----------------------------------------------------------------- hardware
struct Motor {
  uint8_t in1, in2, pwm;
};

// esquerda, direita, traseira
const Motor MOTORES[3] = {
  {  4,  5,  3 },
  {  7,  8,  6 },
  { 12, 13, 11 },
};

// Sinal de cada motor: troque para -1 se a roda girar ao contrário do esperado
const int8_t SENTIDO[3] = { +1, +1, +1 };

// ------------------------------------------------------------- calibração
/**
 * V_MAX_MM_S: velocidade da roda no chão com PWM 100%.
 * Como medir (5 minutos, sem encoder): marque 2 m no chão, coloque o robô para andar
 * reto em PWM máximo e cronometre. v = distância / tempo. Faça 3 vezes e use a média.
 *
 * PWM_MINIMO: abaixo desse valor o motor não vence o atrito estático e só chia.
 * Como medir: suba o PWM de 10 em 10 até a roda começar a girar com o robô no chão.
 *
 * O mapeamento compensa essa zona morta, senão os comandos lentos da fase de frenagem
 * simplesmente não movem o robô:
 *     pwm = PWM_MINIMO + (255 - PWM_MINIMO) * |v| / V_MAX
 */
const float V_MAX_MM_S = 900.0f;
const int   PWM_MINIMO = 45;
const unsigned long TIMEOUT_MS = 200;

// ------------------------------------------------------------------ estado
char  buffer[64];
uint8_t n_buffer = 0;
unsigned long ultimo_quadro_ms = 0;

void aplicar(uint8_t i, float v_mm_s, char modo) {
  const Motor& m = MOTORES[i];

  if (modo == 'I') {                      // inércia: as duas entradas em LOW = coast
    digitalWrite(m.in1, LOW);
    digitalWrite(m.in2, LOW);
    analogWrite(m.pwm, 0);
    return;
  }
  if (modo == 'F') {                      // freio: as duas entradas em HIGH = curto
    digitalWrite(m.in1, HIGH);
    digitalWrite(m.in2, HIGH);
    analogWrite(m.pwm, 255);
    return;
  }

  float v = v_mm_s * SENTIDO[i];
  const bool frente = (v >= 0.0f);
  v = fabs(v);

  int pwm = 0;
  if (v > 1.0f) {                         // ignora comandos residuais
    float f = v / V_MAX_MM_S;
    if (f > 1.0f) f = 1.0f;
    pwm = PWM_MINIMO + (int)((255 - PWM_MINIMO) * f);
  }

  digitalWrite(m.in1, frente ? HIGH : LOW);
  digitalWrite(m.in2, frente ? LOW : HIGH);
  analogWrite(m.pwm, pwm);
}

/** Aciona um motor com PWM cru (-255 a 255). Usado só na calibração. */
void aplicar_pwm(uint8_t i, int pwm) {
  const Motor& m = MOTORES[i];
  pwm *= SENTIDO[i];
  const bool frente = (pwm >= 0);
  if (pwm < 0) pwm = -pwm;
  if (pwm > 255) pwm = 255;
  digitalWrite(m.in1, frente ? HIGH : LOW);
  digitalWrite(m.in2, frente ? LOW : HIGH);
  analogWrite(m.pwm, pwm);
}

void parar_tudo() {
  for (uint8_t i = 0; i < 3; ++i) aplicar(i, 0.0f, 'I');
}

/** Valida o checksum XOR e aplica o quadro. Devolve false se o quadro veio corrompido. */
bool processar(char* linha) {
  char* inicio = strchr(linha, '#');
  char* estrela = strrchr(linha, '*');
  if (!inicio || !estrela || estrela < inicio) return false;

  *estrela = '\0';
  char* corpo = inicio + 1;

  uint8_t chk = 0;
  for (char* p = corpo; *p; ++p) chk ^= (uint8_t)(*p);
  const uint8_t recebido = (uint8_t)strtol(estrela + 1, NULL, 16);
  if (chk != recebido) return false;

  if (corpo[0] == 'P') {                  // quadro de calibração: PWM direto
    int pe = 0, pd = 0, pt = 0;
    if (sscanf(corpo, "P,%d,%d,%d", &pe, &pd, &pt) != 3) return false;
    aplicar_pwm(0, pe);
    aplicar_pwm(1, pd);
    aplicar_pwm(2, pt);
    ultimo_quadro_ms = millis();
    return true;
  }

  if (corpo[0] != 'V') return false;
  int ve = 0, vd = 0, vt = 0;
  char me = 'I', md = 'I', mt = 'I';
  if (sscanf(corpo, "V,%d,%d,%d,%c,%c,%c", &ve, &vd, &vt, &me, &md, &mt) != 6) return false;

  aplicar(0, (float)ve, me);
  aplicar(1, (float)vd, md);
  aplicar(2, (float)vt, mt);
  ultimo_quadro_ms = millis();
  return true;
}

void setup() {
  for (uint8_t i = 0; i < 3; ++i) {
    pinMode(MOTORES[i].in1, OUTPUT);
    pinMode(MOTORES[i].in2, OUTPUT);
    pinMode(MOTORES[i].pwm, OUTPUT);
  }
  parar_tudo();
  Serial.begin(115200);
  ultimo_quadro_ms = millis();
}

void loop() {
  while (Serial.available()) {
    const char c = (char)Serial.read();
    if (c == '\n') {
      buffer[n_buffer] = '\0';
      processar(buffer);
      n_buffer = 0;
    } else if (n_buffer < sizeof(buffer) - 1) {
      buffer[n_buffer++] = c;
    } else {
      n_buffer = 0;                       // quadro maior que o buffer: descarta
    }
  }

  if (millis() - ultimo_quadro_ms > TIMEOUT_MS) {
    parar_tudo();                         // watchdog
  }
}
