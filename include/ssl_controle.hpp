/**
 * ssl_controle.hpp
 *
 * Controle de movimentação e tomada de decisão (FSM) para um robô de 3 rodas:
 *   - 2 rodas laterais (esquerda/direita): translação para frente/trás;
 *   - 1 roda traseira: exclusiva para rotação sobre o próprio eixo.
 * Acionamento intermitente: quando um grupo está ativo, o outro fica em inércia.
 *
 * Convenções (SI em todo o código):
 *   - Distância em metros, ângulo em radianos, tempo em segundos.
 *   - Campo: x para o gol adversário, y para a esquerda, theta anti-horário a partir de +x.
 *   - Robô: x_r para frente (direção de rolagem das rodas laterais), y_r para a esquerda.
 *   - O ponto reportado pela visão e o centro de rotação coincidem com o ponto médio
 *     do eixo das rodas laterais.
 *   - SSL-Vision envia posições em milímetros: divida por 1000 antes de chamar atualizar_visao().
 *
 * Requisitos mecânicos para essa arquitetura funcionar:
 *   - Rodas laterais convencionais (não omni): elas dão a reação lateral durante o giro.
 *   - Roda traseira OMNI montada tangencialmente (rola na direção y_r): assim ela empurra
 *     o giro e, em inércia, seus roletes deixam o robô andar para frente sem arrastar.
 *   - Centro de massa entre o eixo lateral e a roda traseira; caso contrário a roda
 *     traseira fica sem carga normal e não consegue girar o robô (ver calcular_inercia_e_limites).
 *
 * Portável: sem alocação dinâmica, sem exceções, só float. Compila para STM32/ESP32 (com FPU)
 * e pode ser embrulhado em um nó ROS ou no DecisionEngine do projeto SSL.
 */
#pragma once
#include <cstdint>

namespace ssl {

constexpr float PI = 3.14159265358979f;
constexpr float GRAVIDADE = 9.81f;   // m/s^2

inline float grau_para_rad(float g) { return g * PI / 180.0f; }
inline float rad_para_grau(float r) { return r * 180.0f / PI; }

/** Normaliza ângulo para o intervalo [-pi, pi). */
float normalizar_angulo(float a);
/** Satura v no intervalo [minimo, maximo]. */
float limitar(float v, float minimo, float maximo);

/* ------------------------------------------------------------------------ */
/* Parâmetros                                                                */
/* ------------------------------------------------------------------------ */

/** Parâmetros físicos do robô. Valores padrão próximos de um robô SSL típico. */
struct ParametrosFisicos {
    // Massa e geometria
    float massa_kg             = 2.5f;
    float raio_corpo_m         = 0.09f;   // regra SSL: diâmetro máximo de 180 mm
    float altura_cg_m          = 0.04f;   // altura do centro de massa em relação ao chão
    float offset_cg_m          = 0.02f;   // CG atrás do eixo lateral (positivo = para trás)
    float massa_roda_kg        = 0.05f;   // roda + cubo (tratada como massa pontual)

    // Rodas
    float raio_roda_m          = 0.027f;
    float bitola_m             = 0.15f;   // distância entre as rodas laterais
    float dist_roda_tras_m     = 0.07f;   // do centro de rotação até a roda traseira

    // Atrito (coeficiente estático de aderência com o carpete)
    float mu_lateral           = 0.8f;    // borracha
    float mu_traseira          = 0.5f;    // roda omni tem menos aderência

    // Motores (ex.: brushless tipo Maxon EC45 flat 50 W) e redução
    float torque_motor_max_Nm  = 0.10f;   // torque máximo no eixo do motor
    float rpm_motor_max        = 5000.0f; // rotação máxima no eixo do motor
    float reducao              = 3.6f;    // motor gira 3,6x mais rápido que a roda
    float eficiencia           = 0.9f;    // eficiência da transmissão

    // Segurança
    float fator_seguranca      = 0.8f;    // usa só 80% dos limites calculados
    float v_max_estrategia     = 3.0f;    // teto de velocidade linear imposto pela estratégia (m/s)
    float w_max_estrategia     = 10.0f;   // teto de velocidade angular (rad/s)
};

/** Parâmetros da FSM e dos controladores. */
struct ParametrosControle {
    float tol_angulo_rad         = grau_para_rad(2.0f);  // sai do ALINHAMENTO abaixo disso
    float angulo_realinhar_rad   = grau_para_rad(15.0f); // volta a alinhar acima disso
    float w_min_transicao        = 0.3f;   // |w| máximo para trocar de grupo de rodas (rad/s)
    float kp_angulo              = 8.0f;   // ganho P perto do ângulo final (1/s)

    float tol_posicao_m          = 0.02f;  // considera o alvo atingido
    float dist_parada_m          = 0.0f;   // para a esta distância do alvo (ex.: raio robô + raio bola)
    float histerese_reinicio_m   = 0.05f;  // alvo precisa se afastar isso para sair do PARADO
    float kp_linear              = 6.0f;   // ganho P perto do ponto final (1/s)
    float v_min_aproximacao      = 0.05f;  // evita o robô "morrer" antes do alvo (m/s)
    float margem_frenagem        = 1.1f;   // começa a frear 10% antes do necessário
    float atraso_sistema_s       = 0.03f;  // latência visão + resposta do motor (compensada)

    float kp_rumo                = 3.0f;   // correção diferencial de rumo no deslocamento (1/s)
    float w_corr_max             = 2.0f;   // limite da correção de rumo (rad/s)
    float dist_congelar_rumo_m   = 0.10f;  // perto do alvo o atan2 fica ruidoso: congela a correção

    float timeout_visao_s        = 0.10f;  // sem visão por mais que isso: para com rampa
    bool  freio_ativo_parado     = true;   // PARADO: laterais em freio (segura empurrões)
};

/** Geometria do campo (padrão: Divisão A da SSL). */
struct Campo {
    float comprimento_m = 12.0f;
    float largura_m     = 9.0f;
    float borda_m       = 0.3f;    // faixa fora das linhas onde o robô ainda pode andar
};

/** Resultado de calcular_inercia_e_limites(). */
struct LimitesDinamicos {
    float inercia_kgm2      = 0.0f;
    float normal_lateral_N  = 0.0f;  // soma das duas rodas laterais (estático)
    float normal_traseira_N = 0.0f;
    float a_acel_max        = 0.0f;  // m/s^2
    float a_desac_max       = 0.0f;  // m/s^2
    float v_max             = 0.0f;  // m/s
    float alfa_max          = 0.0f;  // rad/s^2
    float w_max             = 0.0f;  // rad/s
    bool  valido            = false;
};

/* ------------------------------------------------------------------------ */
/* Tipos de saída                                                            */
/* ------------------------------------------------------------------------ */

enum class Estado : uint8_t { PARADO, ALINHAMENTO, DESLOCAMENTO, FRENAGEM };

/** ATIVO: segue velocidade. INERCIA: driver em coast (livre). FREIO: driver em curto (brake). */
enum class ModoMotor : uint8_t { ATIVO, INERCIA, FREIO };

enum Roda : uint8_t { RODA_ESQ = 0, RODA_DIR = 1, RODA_TRAS = 2, NUM_RODAS = 3 };

/** Velocidade tangencial de cada roda (m/s) e modo do driver. */
struct ComandoRodas {
    float     v[NUM_RODAS]    = {0.0f, 0.0f, 0.0f};
    ModoMotor modo[NUM_RODAS] = {ModoMotor::INERCIA, ModoMotor::INERCIA, ModoMotor::INERCIA};
};

/** Sinais prontos para o firmware: referência de RPM (para o PID de cada motor) e duty feedforward. */
struct SinaisMotores {
    float     rpm_ref[NUM_RODAS] = {0.0f, 0.0f, 0.0f};  // no eixo do motor
    float     duty[NUM_RODAS]    = {0.0f, 0.0f, 0.0f};  // [-1, 1]
    ModoMotor modo[NUM_RODAS]    = {ModoMotor::INERCIA, ModoMotor::INERCIA, ModoMotor::INERCIA};
};

const char* nome_estado(Estado e);

/* ------------------------------------------------------------------------ */
/* Controlador                                                               */
/* ------------------------------------------------------------------------ */

class ControladorSSL {
public:
    ControladorSSL(const ParametrosFisicos& fis = ParametrosFisicos{},
                   const ParametrosControle& ctrl = ParametrosControle{},
                   const Campo& campo = Campo{});

    /** Física: inércia, forças normais e limites de aceleração/velocidade. */
    LimitesDinamicos calcular_inercia_e_limites();

    /** Chamar a cada frame da visão (~60 Hz). Unidades SI. */
    void atualizar_visao(float x_robo, float y_robo, float theta_robo,
                         float x_alvo, float y_alvo);

    /** FSM. Chamar no laço de controle com período dt (ex.: 200 Hz -> dt = 0.005). */
    ComandoRodas tomar_decisao(float dt);

    /** Converte velocidades tangenciais das rodas em sinais de motor (usa os modos da última decisão). */
    SinaisMotores gerar_sinais_motores(float v_lateral_esq, float v_lateral_dir, float v_traseira) const;

    // Consultas (telemetria / debug)
    Estado estado() const { return estado_; }
    float  distancia() const { return distancia_; }
    float  erro_angulo() const { return erro_angulo_; }
    float  velocidade_linear() const { return v_; }
    float  velocidade_giro() const { return w_giro_; }
    const LimitesDinamicos& limites() const { return limites_; }
    const ComandoRodas& ultimo_comando() const { return comando_; }

private:
    void  calcular_trajetoria();
    float correcao_rumo() const;
    float rampa_linear(float atual, float alvo, float dt) const;
    static float perfil_sqrt(float erro, float a, float kp);
    ComandoRodas parar_com_rampa(float dt);
    ComandoRodas montar_comando();
    static float rampa(float atual, float alvo, float passo_max);

    ParametrosFisicos  fis_;
    ParametrosControle ctrl_;
    Campo              campo_;
    LimitesDinamicos   limites_;

    // Dados da visão
    float x_ = 0.0f, y_ = 0.0f, theta_ = 0.0f;
    float x_alvo_ = 0.0f, y_alvo_ = 0.0f;
    bool  visao_ok_ = false;
    float tempo_sem_visao_ = 0.0f;

    // Trajetória
    float distancia_   = 0.0f;
    float theta_alvo_  = 0.0f;
    float erro_angulo_ = 0.0f;

    // Estado interno
    Estado estado_   = Estado::PARADO;
    bool   realinhar_ = false;  // FRENAGEM causada por erro de rumo grande
    float  d_min_frenagem_ = 0.0f; // menor distância vista na FRENAGEM (detecta alvo fugindo)
    float  v_        = 0.0f;    // velocidade linear comandada (m/s)
    float  w_corr_   = 0.0f;    // correção de rumo pelas laterais (rad/s)
    float  w_giro_   = 0.0f;    // velocidade angular pela roda traseira (rad/s)
    ComandoRodas comando_;
};

}  // namespace ssl
