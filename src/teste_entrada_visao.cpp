/**
 * teste_entrada_visao.cpp
 *
 * Mostra o que a camada de entrada resolve. Roda o mesmo robô, com a mesma visão
 * ruim, de dois jeitos:
 *   A) pose crua da visão entregue direto ao controle;
 *   B) pose passando por EntradaVisao (filtro, predição e rejeição de erro).
 *
 * Visão simulada, com os defeitos que aparecem de verdade:
 *   - 60 Hz, enquanto o controle roda a 200 Hz (2 em cada 3 ciclos ficam sem frame novo);
 *   - 40 ms de latência (a medida que chega agora é de onde o robô estava antes);
 *   - ruído de 3 mm na posição e 1,5 grau no ângulo;
 *   - 5% de frames perdidos;
 *   - 1% de deteções erradas, com salto de cerca de 1 m.
 */
#include "entrada_visao.hpp"
#include "ssl_controle.hpp"

#include <cmath>
#include <cstdio>
#include <deque>
#include <random>

using namespace ssl;

namespace {

struct Planta {
    float x = 0.0f, y = 0.0f, theta = 0.0f, v = 0.0f, w = 0.0f;
    float tau = 0.02f, tau_livre = 0.15f;

    void passo(const ComandoRodas& c, const ParametrosFisicos& p, float dt) {
        if (c.modo[RODA_ESQ] == ModoMotor::ATIVO) {
            const float v_ref = 0.5f * (c.v[RODA_ESQ] + c.v[RODA_DIR]);
            const float w_ref = (c.v[RODA_DIR] - c.v[RODA_ESQ]) / p.bitola_m;
            v += (v_ref - v) * dt / tau;
            w += (w_ref - w) * dt / tau;
        } else if (c.modo[RODA_TRAS] == ModoMotor::ATIVO) {
            const float w_ref = c.v[RODA_TRAS] / p.dist_roda_tras_m;
            w += (w_ref - w) * dt / tau;
            v -= v * dt / tau_livre;
        } else {
            v -= v * dt / tau_livre;
            w -= w * dt / tau_livre;
        }
        x += v * std::cos(theta) * dt;
        y += v * std::sin(theta) * dt;
        theta = normalizar_angulo(theta + w * dt);
    }
};

struct Medida { float x, y, theta, t; };

struct Resultado {
    float erro_rms_mm, erro_max_mm, dist_final, tempo_s;
    int frames, descartados;
};

Resultado rodar(bool usar_filtro, float alvo_x, float alvo_y, unsigned semente) {
    ParametrosFisicos fis;
    ParametrosControle ctrl;
    ctrl.dist_parada_m = 0.11f;
    ControladorSSL robo(fis, ctrl);

    ConfigEntrada cfg;
    cfg.latencia_s = 0.040f;
    EntradaVisao entrada(cfg);

    Planta planta{-1.5f, -0.8f, grau_para_rad(70.0f), 0.0f, 0.0f};

    std::mt19937 rng(semente);
    std::normal_distribution<float> ruido_pos(0.0f, 0.003f);
    std::normal_distribution<float> ruido_ang(0.0f, grau_para_rad(1.5f));
    std::uniform_real_distribution<float> sorteio(0.0f, 1.0f);

    const float dt = 0.005f, periodo_visao = 1.0f / 60.0f, latencia = 0.040f;
    std::deque<Medida> fila;   // medidas esperando a latência passar
    float t = 0.0f, t_visao = 0.0f;
    double soma_erro2 = 0.0;
    float erro_max = 0.0f;
    int amostras = 0, frames = 0;
    float ultima_crua_x = 0.0f, ultima_crua_y = 0.0f;

    for (int k = 0; k < 4000; ++k) {
        t = k * dt;

        // Câmera: amostra a pose verdadeira e guarda com carimbo de tempo
        t_visao += dt;
        if (t_visao >= periodo_visao) {
            t_visao -= periodo_visao;
            if (sorteio(rng) > 0.05f) {           // 5% de frames perdidos
                float mx = planta.x + ruido_pos(rng);
                float my = planta.y + ruido_pos(rng);
                float mt = planta.theta + ruido_ang(rng);
                if (sorteio(rng) < 0.01f) {       // 1% de deteção errada
                    mx += 0.9f;
                    my -= 0.6f;
                }
                fila.push_back({mx, my, mt, t});
            }
        }

        // Entrega do frame ao controle, depois da latência
        while (!fila.empty() && t - fila.front().t >= latencia) {
            const Medida m = fila.front();
            fila.pop_front();
            ++frames;
            if (usar_filtro) {
                entrada.nova_medida(m.x, m.y, m.theta);
            } else {
                robo.atualizar_visao(m.x, m.y, m.theta, alvo_x, alvo_y);
                ultima_crua_x = m.x; ultima_crua_y = m.y;
            }
        }

        // Erro da pose em que o controle acredita, contra onde o robô realmente está
        float cx = 0.0f, cy = 0.0f;
        bool tem_crenca = false;
        if (usar_filtro) {
            entrada.avancar(dt);
            const PoseEstimada& p = entrada.pose();
            if (p.valida) {
                robo.atualizar_visao(p.x, p.y, p.theta, alvo_x, alvo_y);
                cx = p.x; cy = p.y; tem_crenca = true;
            }
        } else if (frames > 0) {
            cx = ultima_crua_x; cy = ultima_crua_y; tem_crenca = true;
        }
        if (tem_crenca) {
            const float e = std::hypot(cx - planta.x, cy - planta.y) * 1000.0f;
            soma_erro2 += (double)e * e;
            erro_max = (e > erro_max) ? e : erro_max;
            ++amostras;
        }

        const ComandoRodas c = robo.tomar_decisao(dt);
        planta.passo(c, fis, dt);

        if (robo.estado() == Estado::PARADO && k > 200) break;
    }

    Resultado r{};
    r.erro_rms_mm = amostras ? (float)std::sqrt(soma_erro2 / amostras) : 0.0f;
    r.erro_max_mm = erro_max;
    r.dist_final = std::hypot(alvo_x - planta.x, alvo_y - planta.y);
    r.tempo_s = t;
    r.frames = frames;
    r.descartados = entrada.medidas_descartadas();
    return r;
}

}  // namespace

int main() {
    const float alvo_x = 1.4f, alvo_y = 0.9f;
    const float esperado = 0.11f;

    std::printf("Alvo (%.2f, %.2f), parar a %.2f m dele. Visao: 60 Hz, 40 ms de atraso,\n"
                "3 mm de ruido, 5%% de frames perdidos, 1%% de detecao errada.\n\n",
                alvo_x, alvo_y, esperado);

    float parada_a = 0.0f, parada_b = 0.0f, rms_a = 0.0f, rms_b = 0.0f;
    float pior_a = 0.0f, pior_b = 0.0f, tempo_a = 0.0f, tempo_b = 0.0f;
    const int n = 20;
    for (unsigned s = 1; s <= (unsigned)n; ++s) {
        const Resultado a = rodar(false, alvo_x, alvo_y, s);
        const Resultado b = rodar(true, alvo_x, alvo_y, s);
        parada_a += std::fabs(a.dist_final - esperado);  parada_b += std::fabs(b.dist_final - esperado);
        rms_a += a.erro_rms_mm;                          rms_b += b.erro_rms_mm;
        pior_a = (a.erro_max_mm > pior_a) ? a.erro_max_mm : pior_a;
        pior_b = (b.erro_max_mm > pior_b) ? b.erro_max_mm : pior_b;
        tempo_a += a.tempo_s;                            tempo_b += b.tempo_s;
    }
    std::printf("%-22s %12s %12s %12s %10s\n", "", "erro RMS", "pior erro", "erro parada", "tempo");
    std::printf("%-22s %9.1f mm %9.1f mm %9.1f mm %8.2f s\n", "A) pose crua",
                rms_a / n, pior_a, parada_a / n * 1000.0f, tempo_a / n);
    std::printf("%-22s %9.1f mm %9.1f mm %9.1f mm %8.2f s\n", "B) com EntradaVisao",
                rms_b / n, pior_b, parada_b / n * 1000.0f, tempo_b / n);

    const Resultado b = rodar(true, alvo_x, alvo_y, 7);
    std::printf("\nCorrida de exemplo (semente 7): %d frames recebidos, "
                "%d deteccoes erradas descartadas\n", b.frames, b.descartados);
    std::printf("Erro RMS da estimativa: %.1f mm | distancia final ao alvo: %.3f m\n",
                b.erro_rms_mm, b.dist_final);
    std::printf("\nerro RMS  = o quanto a pose em que o controle acredita difere da real\n"
                "erro parada = |distancia final - distancia pedida|\n");
    return 0;
}
