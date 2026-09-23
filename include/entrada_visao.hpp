/**
 * entrada_visao.hpp
 *
 * Camada entre o que o grupo da visão entrega e o que o controle precisa.
 *
 * O controle precisa da pose AGORA, a 200 Hz, em metros e radianos, sem saltos.
 * A visão entrega a pose de ALGUNS MILISSEGUNDOS ATRÁS, a 30 ou 60 Hz, com ruído,
 * com frames perdidos e, de vez em quando, com uma deteção errada. Esta camada
 * resolve os quatro problemas:
 *
 *   1. Unidades e convenções: a visão pode mandar milímetros e graus, com outra
 *      origem ou outro sentido de y. Converte tudo aqui, num lugar só.
 *   2. Latência: exposição, processamento e rede somam atraso. A medida vale para
 *      t - latencia. A pose é projetada para frente usando a velocidade estimada.
 *      A 2 m/s, 40 ms de atraso são 8 cm de erro, o que sozinho já estraga a frenagem.
 *   3. Ruído e buracos entre frames: filtro alfa-beta estima posição e velocidade e
 *      preenche os ciclos em que não chegou frame novo (dead reckoning curto).
 *   4. Deteção errada: salto maior que o fisicamente possível é descartado. Se o
 *      salto se repetir, a medida é aceita (o robô pode ter sido pego e movido).
 *
 * Nada aqui depende do transporte. Seja UDP, protobuf, memcached ou ROS, o grupo da
 * visão entrega números e esta classe cuida do resto.
 */
#pragma once

#include <cstdint>

namespace ssl {

/** Como interpretar o que a visão manda. Preencher conforme o contrato acertado. */
struct ConfigEntrada {
    // Unidades
    float escala_posicao   = 1.0f;    // 1.0 se vier em metros, 0.001 se vier em milímetros
    bool  angulo_em_graus  = false;   // true se theta vier em graus
    bool  inverter_y       = false;   // true se o y da visão cresce para o outro lado
    float origem_x         = 0.0f;    // deslocamento da origem da visão até a origem do controle (m)
    float origem_y         = 0.0f;
    float offset_theta_rad = 0.0f;    // se o zero de ângulo da visão não é o nosso zero

    // Tempo
    float latencia_s       = 0.040f;  // atraso total medido: exposição + processamento + rede
    float timeout_s        = 0.150f;  // sem frame por mais que isso, a pose vira inválida

    // Filtro alfa-beta (0 = ignora a medida, 1 = confia só na medida)
    // Valores ajustados por varredura contra visão de 60 Hz com 3 mm de ruído
    float alfa             = 0.45f;   // correção de posição
    float beta             = 0.30f;   // correção de velocidade
    float alfa_angulo      = 0.55f;
    float beta_angulo      = 0.30f;

    // Rejeição de deteção errada
    float salto_max_m      = 0.30f;   // salto plausível entre dois frames a 60 Hz
    float salto_max_rad    = 1.60f;
    int   saltos_para_aceitar = 3;    // depois de N saltos seguidos, aceita e reinicia
};

/** Pose estimada no instante atual, já no referencial e nas unidades do controle. */
struct PoseEstimada {
    float x = 0.0f, y = 0.0f, theta = 0.0f;
    float vx = 0.0f, vy = 0.0f, omega = 0.0f;   // estimados pelo filtro
    bool  valida = false;
    float idade_s = 0.0f;                        // desde o último frame aceito
};

class EntradaVisao {
public:
    explicit EntradaVisao(const ConfigEntrada& cfg = ConfigEntrada{});

    /**
     * Chamar quando chegar um frame da visão, com os valores crus, nas unidades dela.
     * Devolve false se a medida foi descartada como deteção errada.
     */
    bool nova_medida(float x_cru, float y_cru, float theta_cru);

    /** Chamar todo ciclo de controle. Propaga a estimativa e envelhece a medida. */
    void avancar(float dt);

    /** Converte um ponto qualquer da visão (a bola, um obstáculo) para o referencial do controle. */
    void converter_ponto(float x_cru, float y_cru, float& x, float& y) const;

    const PoseEstimada& pose() const { return pose_; }
    int  medidas_aceitas() const { return aceitas_; }
    int  medidas_descartadas() const { return descartadas_; }
    void reiniciar();

private:
    void converter(float x_cru, float y_cru, float theta_cru,
                   float& x, float& y, float& theta) const;

    ConfigEntrada cfg_;
    PoseEstimada  pose_;
    bool  iniciado_ = false;
    int   saltos_seguidos_ = 0;
    int   aceitas_ = 0, descartadas_ = 0;
    float dt_medida_ = 1.0f / 60.0f;   // intervalo estimado entre frames
    float t_desde_medida_ = 0.0f;
};

}  // namespace ssl
