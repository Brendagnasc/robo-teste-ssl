/**
 * main_simulacao.cpp
 * Teste em malha fechada no PC: planta cinemática simples + visão a 60 Hz com ruído
 * + controle a 200 Hz. Serve para validar a FSM antes de ir para o grSim ou o robô real.
 *
 * Integração real (mesma estrutura):
 *   STM32/ESP32: timer de 200 Hz chama tomar_decisao(); callback do rádio chama atualizar_visao().
 *   ROS:         subscriber da visão -> atualizar_visao(); timer -> tomar_decisao() -> publica comando.
 */
#include "ssl_controle.hpp"
#include <cmath>
#include <cstdio>
#include <random>

using namespace ssl;

/** Planta: rodas seguem o comando com atraso de 1a ordem; em inércia, desaceleram por atrito. */
struct PlantaRobo {
    float x, y, theta;
    float v = 0.0f, w = 0.0f;           // velocidades reais do corpo
    float tau_motor = 0.02f;            // constante de tempo dos motores (s)
    float tau_inercia = 0.15f;          // decaimento livre por atrito (s)
    float tau_freio = 0.03f;

    void passo(const ComandoRodas& c, const ParametrosFisicos& p, float dt) {
        const bool laterais = c.modo[RODA_ESQ] == ModoMotor::ATIVO;
        const bool traseira = c.modo[RODA_TRAS] == ModoMotor::ATIVO;
        const bool freio    = c.modo[RODA_ESQ] == ModoMotor::FREIO;

        if (laterais) {
            const float v_ref = 0.5f * (c.v[RODA_ESQ] + c.v[RODA_DIR]);
            const float w_ref = (c.v[RODA_DIR] - c.v[RODA_ESQ]) / p.bitola_m;
            v += (v_ref - v) * dt / tau_motor;
            w += (w_ref - w) * dt / tau_motor;
        } else if (traseira) {
            const float w_ref = c.v[RODA_TRAS] / p.dist_roda_tras_m;
            w += (w_ref - w) * dt / tau_motor;
            v -= v * dt / tau_inercia;
        } else {
            const float tau = freio ? tau_freio : tau_inercia;
            v -= v * dt / tau;
            w -= w * dt / tau;
        }
        x += v * std::cos(theta) * dt;
        y += v * std::sin(theta) * dt;
        theta = normalizar_angulo(theta + w * dt);
    }
};

int main() {
    ParametrosFisicos fis;
    ParametrosControle ctrl;
    ctrl.dist_parada_m = fis.raio_corpo_m + 0.0215f;   // para encostado na bola (raio 21,5 mm)
    ControladorSSL robo(fis, ctrl);

    const LimitesDinamicos& L = robo.limites();
    std::printf("=== Limites calculados (valido=%d) ===\n", L.valido);
    std::printf("I = %.5f kg.m^2 | N_lat = %.2f N | N_tras = %.2f N\n",
                L.inercia_kgm2, L.normal_lateral_N, L.normal_traseira_N);
    std::printf("a_acel = %.2f m/s^2 | a_desac = %.2f m/s^2 | v_max = %.2f m/s\n",
                L.a_acel_max, L.a_desac_max, L.v_max);
    std::printf("alfa = %.2f rad/s^2 | w_max = %.2f rad/s\n\n", L.alfa_max, L.w_max);

    PlantaRobo planta{-2.0f, -1.0f, grau_para_rad(90.0f)};
    std::mt19937 rng(42);
    std::normal_distribution<float> ruido_pos(0.0f, 0.002f);   // 2 mm
    std::normal_distribution<float> ruido_ang(0.0f, grau_para_rad(0.3f));

    const float dt = 0.005f;            // controle a 200 Hz
    const float periodo_visao = 1.0f / 60.0f;
    float t_visao = 0.0f;
    float alvo_x = 1.5f, alvo_y = 1.0f; // bola
    Estado anterior = Estado::PARADO;
    bool trocou_alvo = false;
    int k_troca = 0;

    std::printf("   t(s)  estado        x      y    theta(g)  d(m)   erro(g)  v(m/s)  w(rad/s)\n");
    for (int k = 0; k < 4000; ++k) {
        const float t = k * dt;

        // Depois de chegar, a bola "vai" para trás do robô (exige giro grande)
        if (!trocou_alvo && t > 0.5f && robo.estado() == Estado::PARADO && k > 200) {
            alvo_x = -1.0f; alvo_y = 0.5f; trocou_alvo = true; k_troca = k;
            std::printf("--- novo alvo (%.2f, %.2f) ---\n", alvo_x, alvo_y);
        }

        t_visao += dt;
        if (t_visao >= periodo_visao) {
            t_visao -= periodo_visao;
            robo.atualizar_visao(planta.x + ruido_pos(rng), planta.y + ruido_pos(rng),
                                 planta.theta + ruido_ang(rng), alvo_x, alvo_y);
        }

        const ComandoRodas c = robo.tomar_decisao(dt);
        const SinaisMotores s = robo.gerar_sinais_motores(c.v[RODA_ESQ], c.v[RODA_DIR], c.v[RODA_TRAS]);
        (void)s;   // no robô real: s vai para o driver (PWM/PID)
        planta.passo(c, fis, dt);

        if (robo.estado() != anterior || k % 40 == 0) {
            std::printf("%7.3f  %-12s %6.3f %6.3f %8.2f %6.3f %8.2f %7.3f %8.3f\n",
                        t, nome_estado(robo.estado()), planta.x, planta.y,
                        rad_para_grau(planta.theta), robo.distancia(),
                        rad_para_grau(robo.erro_angulo()), planta.v, planta.w);
            anterior = robo.estado();
        }
        if (trocou_alvo && k - k_troca > 40 && robo.estado() == Estado::PARADO) break;
    }

    const float dx = alvo_x - planta.x, dy = alvo_y - planta.y;
    std::printf("\nDistancia final ao alvo: %.3f m (esperado ~%.3f)\n",
                std::sqrt(dx * dx + dy * dy), ctrl.dist_parada_m);
    return 0;
}
