/**
 * ssl_controle.cpp
 * Implementação da física, da FSM e da geração de sinais. Ver ssl_controle.hpp.
 */
#include "ssl_controle.hpp"
#include <cmath>

namespace ssl {

namespace {
constexpr float MUITO_GRANDE = 1.0e9f;

inline float minimo(float a, float b) { return a < b ? a : b; }
inline float maximo(float a, float b) { return a > b ? a : b; }
inline float sinal(float a) { return (a > 0.0f) ? 1.0f : ((a < 0.0f) ? -1.0f : 0.0f); }
}  // namespace

float normalizar_angulo(float a) {
    a = std::fmod(a + PI, 2.0f * PI);
    if (a < 0.0f) a += 2.0f * PI;
    return a - PI;
}

float limitar(float v, float minimo_, float maximo_) {
    return (v < minimo_) ? minimo_ : ((v > maximo_) ? maximo_ : v);
}

const char* nome_estado(Estado e) {
    switch (e) {
        case Estado::PARADO:       return "PARADO";
        case Estado::ALINHAMENTO:  return "ALINHAMENTO";
        case Estado::DESLOCAMENTO: return "DESLOCAMENTO";
        case Estado::FRENAGEM:     return "FRENAGEM";
    }
    return "?";
}

ControladorSSL::ControladorSSL(const ParametrosFisicos& fis,
                               const ParametrosControle& ctrl,
                               const Campo& campo)
    : fis_(fis), ctrl_(ctrl), campo_(campo) {
    calcular_inercia_e_limites();
    montar_comando();
}

/* ========================================================================
 * FÍSICA
 * ========================================================================
 *
 * Vista lateral (robô andando para a direita):
 *
 *          CG (altura h)
 *           o
 *   --------|-------------------
 *   (T)     |       (L)            T = roda traseira, L = eixo das laterais
 *    |<-Lt->|<-xcg->|             (xcg medido do eixo L para trás)
 *
 * 1) Momento de inércia em torno do centro de rotação (eixo das laterais):
 *      corpo como disco sólido:  I_cm = 1/2 * m * R^2
 *      Steiner (eixos paralelos): I = I_cm + m * xcg^2
 *      rodas como massas pontuais: I += sum(m_roda * r_i^2)
 *
 * 2) Forças normais (equilíbrio de momentos, robô parado):
 *      N_tras = P * xcg / Lt        N_lat = P - N_tras
 *    Se xcg <= 0, a roda traseira não tem carga e não consegue girar o robô.
 *
 * 3) Aceleração linear máxima (limite por aderência COM transferência de carga):
 *    Acelerando, a força inercial (m*a na altura h) tira carga das laterais:
 *      N_lat = m*(g*(Lt - xcg) - a*h) / Lt   e   a = mu * N_lat / m
 *      => a_tracao = mu*g*(Lt - xcg) / (Lt + mu*h)
 *    Limite do motor: a_motor = 2 * F_roda / m,  F_roda = tau*reducao*eta / r
 *
 * 4) Desaceleração máxima: carga vai para frente (sobre as laterais), mas não há
 *    roda na frente. Se a*h > g*xcg a roda traseira descola e o robô "pica":
 *      a_tombamento = g * xcg / h
 *    Aderência na frenagem: a = mu*g*(Lt - xcg) / (Lt - mu*h)
 *
 * 5) Rotação: a roda traseira empurra tangencialmente a um braço Lt:
 *      F_tras = min(mu_tras * N_tras, F_roda)     alfa = F_tras * Lt / I
 *    Velocidade angular: limite do motor (v_roda_max / Lt) e da força centrípeta
 *    que as laterais precisam fornecer ao CG: m*w^2*xcg <= mu*N_lat
 *
 * O fator de segurança garante folga para que a aceleração CONSTANTE escolhida
 * nunca faça as rodas patinarem (patinar = perda de controle e desgaste).
 */
LimitesDinamicos ControladorSSL::calcular_inercia_e_limites() {
    const ParametrosFisicos& p = fis_;
    LimitesDinamicos L{};

    const float Lt   = p.dist_roda_tras_m;
    const float xcg  = p.offset_cg_m;
    const float h    = p.altura_cg_m;
    const float fs   = p.fator_seguranca;
    const float peso = p.massa_kg * GRAVIDADE;

    // (1) Momento de inércia
    const float m_rodas    = 3.0f * p.massa_roda_kg;
    const float m_corpo    = p.massa_kg - m_rodas;
    const float I_corpo_cm = 0.5f * m_corpo * p.raio_corpo_m * p.raio_corpo_m;
    const float I_corpo    = I_corpo_cm + m_corpo * xcg * xcg;
    const float meia_bitola = 0.5f * p.bitola_m;
    const float I_rodas    = 2.0f * p.massa_roda_kg * meia_bitola * meia_bitola
                           + p.massa_roda_kg * Lt * Lt;
    L.inercia_kgm2 = I_corpo + I_rodas;

    // Validação geométrica: CG precisa estar entre o eixo lateral e a roda traseira
    if (xcg <= 0.0f || xcg >= Lt || h <= 0.0f || m_corpo <= 0.0f) {
        L.valido = false;   // robô não se move com limites inválidos
        limites_ = L;
        return L;
    }

    // (2) Forças normais estáticas
    L.normal_traseira_N = peso * xcg / Lt;
    L.normal_lateral_N  = peso - L.normal_traseira_N;

    // Capacidade do motor refletida na roda
    const float w_motor_max = p.rpm_motor_max * 2.0f * PI / 60.0f;          // rad/s no motor
    const float v_roda_max  = (w_motor_max / p.reducao) * p.raio_roda_m;    // m/s no chão
    const float F_roda      = p.torque_motor_max_Nm * p.reducao * p.eficiencia / p.raio_roda_m;

    // (3) Aceleração
    const float a_tracao_acel = p.mu_lateral * GRAVIDADE * (Lt - xcg) / (Lt + p.mu_lateral * h);
    const float a_motor       = 2.0f * F_roda / p.massa_kg;
    L.a_acel_max = fs * minimo(a_tracao_acel, a_motor);

    // (4) Desaceleração
    const float den = Lt - p.mu_lateral * h;
    const float a_tracao_desac = (den > 0.0f)
        ? p.mu_lateral * GRAVIDADE * (Lt - xcg) / den
        : MUITO_GRANDE;                                   // aderência não limita
    const float a_tombamento = GRAVIDADE * xcg / h;
    L.a_desac_max = fs * minimo(minimo(a_tracao_desac, a_motor), a_tombamento);

    L.v_max = minimo(fs * v_roda_max, p.v_max_estrategia);

    // (5) Rotação
    const float F_tras = minimo(p.mu_traseira * L.normal_traseira_N, F_roda);
    L.alfa_max = fs * F_tras * Lt / L.inercia_kgm2;

    const float w_motor      = v_roda_max / Lt;
    const float w_centripeta = std::sqrt(p.mu_lateral * L.normal_lateral_N / (p.massa_kg * xcg));
    L.w_max = minimo(fs * minimo(w_motor, w_centripeta), p.w_max_estrategia);

    L.valido = true;
    limites_ = L;
    return L;
}

/* ========================================================================
 * VISÃO
 * ======================================================================== */
void ControladorSSL::atualizar_visao(float x_robo, float y_robo, float theta_robo,
                                     float x_alvo, float y_alvo) {
    x_ = x_robo;
    y_ = y_robo;
    theta_ = normalizar_angulo(theta_robo);

    // Alvo limitado à área onde o robô cabe (campo + borda - raio do robô)
    const float lim_x = 0.5f * campo_.comprimento_m + campo_.borda_m - fis_.raio_corpo_m;
    const float lim_y = 0.5f * campo_.largura_m     + campo_.borda_m - fis_.raio_corpo_m;
    x_alvo_ = limitar(x_alvo, -lim_x, lim_x);
    y_alvo_ = limitar(y_alvo, -lim_y, lim_y);

    visao_ok_ = true;
    tempo_sem_visao_ = 0.0f;
    calcular_trajetoria();
}

/**
 * Trajetória em linha reta:
 *   d = sqrt(dx^2 + dy^2)
 *   theta_alvo = atan2(dy, dx)
 *   theta_erro = normalizar(theta_alvo - theta_robo)   em [-pi, pi)
 * Normalizar evita que o robô gire 350 graus quando bastariam 10 no sentido oposto.
 */
void ControladorSSL::calcular_trajetoria() {
    const float dx = x_alvo_ - x_;
    const float dy = y_alvo_ - y_;
    distancia_ = std::sqrt(dx * dx + dy * dy);
    if (distancia_ > 1.0e-4f) {                 // em cima do alvo o atan2 é indefinido
        theta_alvo_ = std::atan2(dy, dx);
    }
    erro_angulo_ = normalizar_angulo(theta_alvo_ - theta_);
}

/* ========================================================================
 * RAMPAS
 * ======================================================================== */

/** Limita a variação por passo: |nova - atual| <= passo_max. É o que impõe aceleração constante. */
float ControladorSSL::rampa(float atual, float alvo, float passo_max) {
    if (alvo > atual + passo_max) return atual + passo_max;
    if (alvo < atual - passo_max) return atual - passo_max;
    return alvo;
}

/** Rampa linear com limites diferentes para acelerar e frear. */
float ControladorSSL::rampa_linear(float atual, float alvo, float dt) const {
    const bool acelerando = std::fabs(alvo) > std::fabs(atual) && sinal(alvo) * sinal(atual) >= 0.0f;
    const float a = acelerando ? limites_.a_acel_max : limites_.a_desac_max;
    return rampa(atual, alvo, a * dt);
}

/**
 * Perfil de velocidade "sqrt controller" (mesma ideia usada no ArduPilot):
 *   erro grande:  v = sqrt(2*a*(erro - a/(2*kp^2)))   -> desaceleração exatamente a
 *   erro pequeno: v = kp*erro                          -> chegada suave, sem oscilar
 * As duas partes se encontram em erro = a/kp^2 com mesma velocidade, então a curva é
 * contínua e a desaceleração exigida nunca passa de a (a rampa consegue acompanhar).
 * Um simples min(sqrt(2ad), kp*d) exigiria até 2a na junção e o robô passaria do alvo.
 */
float ControladorSSL::perfil_sqrt(float erro, float a, float kp) {
    if (erro <= 0.0f) return 0.0f;
    const float juncao = a / (kp * kp);
    if (erro <= juncao) return kp * erro;
    return std::sqrt(2.0f * a * (erro - 0.5f * juncao));
}

/**
 * Correção de rumo durante o deslocamento, feita pela diferença de velocidade das
 * laterais (roda traseira omni em inércia deixa o robô fazer curvas suaves).
 * Limitada a 50% da velocidade linear para nenhuma roda inverter: sem velocidade,
 * sem correção (quem gira parado é só a roda traseira).
 */
float ControladorSSL::correcao_rumo() const {
    if (distancia_ < ctrl_.dist_congelar_rumo_m) return 0.0f;
    const float lim_vel = 0.5f * std::fabs(v_) / (0.5f * fis_.bitola_m);
    const float lim = minimo(ctrl_.w_corr_max, lim_vel);
    return limitar(ctrl_.kp_rumo * erro_angulo_, -lim, lim);
}

/* ========================================================================
 * FSM
 * ========================================================================
 *
 *  PARADO --(alvo longe)--> ALINHAMENTO --(|erro|<2 graus)--> DESLOCAMENTO
 *     ^                                                           |
 *     |                                     (dist <= dist. de frenagem)
 *     |                                                           v
 *     +------------(v = 0, no alvo)--------------------------- FRENAGEM
 *                                                                 |
 *   ALINHAMENTO <--------(v = 0, erro de rumo > 15 graus)---------+
 *
 * Perfil de velocidade (trapezoidal): acelera com a constante até v_max, cruza,
 * e freia com a constante. Referência de frenagem: v_ref = sqrt(2*a*d), que é a
 * velocidade da qual se para exatamente em d (Torricelli: v^2 = 2*a*d). Perto do
 * alvo o perfil vira linear (ver perfil_sqrt) para chegar suave, sem oscilação.
 * O mesmo vale para o giro com alfa e o erro angular.
 */
ComandoRodas ControladorSSL::tomar_decisao(float dt) {
    if (dt <= 0.0f) dt = 1.0e-3f;
    tempo_sem_visao_ += dt;

    // Segurança: limites inválidos ou visão perdida -> para suavemente
    if (!limites_.valido || !visao_ok_ || tempo_sem_visao_ > ctrl_.timeout_visao_s) {
        return parar_com_rampa(dt);
    }

    const float d_rest = distancia_ - ctrl_.dist_parada_m;   // quanto falta andar
    const float e      = erro_angulo_;

    switch (estado_) {

    case Estado::PARADO:
        v_ = 0.0f; w_corr_ = 0.0f; w_giro_ = 0.0f;
        if (d_rest > ctrl_.tol_posicao_m + ctrl_.histerese_reinicio_m) {
            estado_ = Estado::ALINHAMENTO;
        }
        break;

    case Estado::ALINHAMENTO: {
        // Laterais em inércia, roda traseira gira o robô
        if (std::fabs(e) < ctrl_.tol_angulo_rad && std::fabs(w_giro_) < ctrl_.w_min_transicao) {
            w_giro_ = 0.0f;
            estado_ = (d_rest > ctrl_.tol_posicao_m) ? Estado::DESLOCAMENTO : Estado::PARADO;
            break;
        }
        // Erro efetivo: desconta o quanto o robô ainda vai girar durante a latência
        const float ae = std::fabs(e) - std::fabs(w_giro_) * ctrl_.atraso_sistema_s;
        const float alfa_perfil = limites_.alfa_max / ctrl_.margem_frenagem;
        const float w_ref = sinal(e) * minimo(limites_.w_max,
                                              perfil_sqrt(ae, alfa_perfil, ctrl_.kp_angulo));
        w_giro_ = rampa(w_giro_, w_ref, limites_.alfa_max * dt);   // alfa constante
        break;
    }

    case Estado::DESLOCAMENTO: {
        // Roda traseira em inércia, laterais aceleram com a constante
        if (std::fabs(e) > ctrl_.angulo_realinhar_rad) {
            realinhar_ = true;               // precisa parar antes de trocar de rodas
            estado_ = Estado::FRENAGEM;
            d_min_frenagem_ = d_rest;
            break;
        }
        v_ = rampa_linear(v_, limites_.v_max, dt);
        w_corr_ = correcao_rumo();

        const float d_frenagem = ctrl_.margem_frenagem * (v_ * v_) / (2.0f * limites_.a_desac_max)
                               + v_ * ctrl_.atraso_sistema_s + ctrl_.tol_posicao_m;
        if (d_rest <= d_frenagem) {
            estado_ = Estado::FRENAGEM;
            d_min_frenagem_ = d_rest;
        }
        break;
    }

    case Estado::FRENAGEM: {
        float v_ref = 0.0f;
        if (!realinhar_ && d_rest > ctrl_.tol_posicao_m) {
            const float d_eff   = d_rest - v_ * ctrl_.atraso_sistema_s;   // compensa latência
            const float a_perfil = limites_.a_desac_max / ctrl_.margem_frenagem;
            v_ref = minimo(limites_.v_max, perfil_sqrt(d_eff, a_perfil, ctrl_.kp_linear));
            v_ref = maximo(v_ref, ctrl_.v_min_aproximacao);

            // Alvo (bola) se afastou: a distância voltou a crescer além da histerese
            d_min_frenagem_ = minimo(d_min_frenagem_, d_rest);
            if (d_rest > d_min_frenagem_ + ctrl_.histerese_reinicio_m) {
                estado_ = Estado::DESLOCAMENTO;
            }
            if (std::fabs(e) > ctrl_.angulo_realinhar_rad) realinhar_ = true;
        }
        v_ = rampa_linear(v_, v_ref, dt);
        w_corr_ = correcao_rumo();

        if (v_ == 0.0f && v_ref == 0.0f) {
            estado_ = realinhar_ ? Estado::ALINHAMENTO : Estado::PARADO;
            realinhar_ = false;
            w_corr_ = 0.0f;
        }
        break;
    }
    }

    return montar_comando();
}

/** Visão perdida: reduz a velocidade do grupo de rodas ativo com a rampa e entra em PARADO. */
ComandoRodas ControladorSSL::parar_com_rampa(float dt) {
    realinhar_ = false;
    if (estado_ == Estado::ALINHAMENTO) {
        w_giro_ = rampa(w_giro_, 0.0f, limites_.alfa_max * dt);
        if (w_giro_ == 0.0f) estado_ = Estado::PARADO;
    } else if (estado_ == Estado::DESLOCAMENTO || estado_ == Estado::FRENAGEM) {
        estado_ = Estado::FRENAGEM;
        v_ = rampa(v_, 0.0f, limites_.a_desac_max * dt);
        w_corr_ = 0.0f;
        if (v_ == 0.0f) estado_ = Estado::PARADO;
    }
    if (!limites_.valido) { v_ = 0.0f; w_giro_ = 0.0f; estado_ = Estado::PARADO; }
    return montar_comando();
}

/**
 * Cinemática inversa de cada modo:
 *   Laterais (diferencial):  v_esq = v - w*b/2    v_dir = v + w*b/2
 *   Traseira (tangencial):   v_tras = w * Lt   (v_tras positivo gira o robô no sentido
 *                            anti-horário; ajuste o sinal no firmware conforme a montagem)
 */
ComandoRodas ControladorSSL::montar_comando() {
    ComandoRodas c;
    const float meia_bitola = 0.5f * fis_.bitola_m;

    switch (estado_) {
    case Estado::ALINHAMENTO:
        c.modo[RODA_ESQ]  = ModoMotor::INERCIA;
        c.modo[RODA_DIR]  = ModoMotor::INERCIA;
        c.modo[RODA_TRAS] = ModoMotor::ATIVO;
        c.v[RODA_TRAS]    = w_giro_ * fis_.dist_roda_tras_m;
        break;

    case Estado::DESLOCAMENTO:
    case Estado::FRENAGEM:
        c.modo[RODA_ESQ]  = ModoMotor::ATIVO;
        c.modo[RODA_DIR]  = ModoMotor::ATIVO;
        c.modo[RODA_TRAS] = ModoMotor::INERCIA;
        c.v[RODA_ESQ]     = v_ - w_corr_ * meia_bitola;
        c.v[RODA_DIR]     = v_ + w_corr_ * meia_bitola;
        break;

    case Estado::PARADO: {
        const ModoMotor m = ctrl_.freio_ativo_parado ? ModoMotor::FREIO : ModoMotor::INERCIA;
        c.modo[RODA_ESQ]  = m;
        c.modo[RODA_DIR]  = m;
        c.modo[RODA_TRAS] = ModoMotor::INERCIA;
        break;
    }
    }
    comando_ = c;
    return c;
}

/* ========================================================================
 * SINAIS DOS MOTORES
 * ========================================================================
 * v (m/s no chão) -> w_roda = v / r -> w_motor = w_roda * reducao -> RPM.
 * rpm_ref é o setpoint do PID de velocidade que roda no firmware (com encoder).
 * duty é só o feedforward (proporcional ao RPM, pois a tensão ~ força contraeletromotriz
 * ~ velocidade); o PID corrige carga e atrito.
 */
SinaisMotores ControladorSSL::gerar_sinais_motores(float v_lateral_esq, float v_lateral_dir,
                                                   float v_traseira) const {
    SinaisMotores s;
    const float v[NUM_RODAS] = {v_lateral_esq, v_lateral_dir, v_traseira};
    const float fator_rpm = fis_.reducao * 60.0f / (2.0f * PI * fis_.raio_roda_m);

    for (int i = 0; i < NUM_RODAS; ++i) {
        s.modo[i] = comando_.modo[i];
        if (s.modo[i] != ModoMotor::ATIVO) continue;     // inércia/freio: sem PWM
        s.rpm_ref[i] = limitar(v[i] * fator_rpm, -fis_.rpm_motor_max, fis_.rpm_motor_max);
        s.duty[i]    = s.rpm_ref[i] / fis_.rpm_motor_max;
    }
    return s;
}

}  // namespace ssl
