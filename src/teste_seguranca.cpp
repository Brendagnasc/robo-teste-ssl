/**
 * teste_seguranca.cpp
 *
 * Confere as proteções contra o robô "fugir" quando o laço visão -> controle quebra
 * sem que os frames parem de chegar:
 *   - visão travada repetindo a última pose (EntradaVisao: frames idênticos);
 *   - robô preso ou motor ligado ao contrário (ControladorSSL: checagem de movimento).
 *
 * O laço é o mesmo da ponte_serial: visão a 60 Hz com 40 ms de latência e ruído,
 * EntradaVisao, controle a 200 Hz. Devolve 0 se todos os cenários passam.
 */
#include "entrada_visao.hpp"
#include "ssl_controle.hpp"

#include <cmath>
#include <cstdio>
#include <deque>
#include <random>

using namespace ssl;

namespace {

enum class Cenario { NORMAL, CONGELA_ANDANDO, CONGELA_GIRANDO, PRESO, MOTOR_INVERTIDO, PARADO_ESTAVEL };

struct Planta {
    float x = 0.0f, y = 0.0f, theta = 0.0f, v = 0.0f, w = 0.0f;
    float tau = 0.02f, tau_livre = 0.15f;
    bool  preso = false, invertido = false;

    void passo(const ComandoRodas& c, const ParametrosFisicos& p, float dt) {
        if (c.modo[RODA_ESQ] == ModoMotor::ATIVO) {
            float v_ref = 0.5f * (c.v[RODA_ESQ] + c.v[RODA_DIR]);
            const float w_ref = (c.v[RODA_DIR] - c.v[RODA_ESQ]) / p.bitola_m;
            if (invertido) v_ref = -v_ref;
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
        if (preso) { v = 0.0f; w = 0.0f; }
        x += v * std::cos(theta) * dt;
        y += v * std::sin(theta) * dt;
        theta = normalizar_angulo(theta + w * dt);
    }
};

struct Medida { float x, y, theta, t; };

struct Resultado {
    bool  passou = false;
    float t_congelou = -1.0f;     // instante em que a visão travou
    float t_parou = -1.0f;        // primeiro instante parado depois do problema
    float andou_depois_m = 0.0f;  // deslocamento real depois que a visão travou
    float v_max_depois = 0.0f;    // maior velocidade real depois de parar (religou?)
    int   congelados = 0, falhas = 0;
    float dist_final = 0.0f;
    const char* motivo = "";
};

Resultado rodar(Cenario cen) {
    ParametrosFisicos fis;
    ParametrosControle ctrl;
    ctrl.dist_parada_m = 0.11f;
    ControladorSSL robo(fis, ctrl);
    ConfigEntrada cfg;
    cfg.latencia_s = 0.040f;
    EntradaVisao entrada(cfg);

    const float alvo_x = 1.4f, alvo_y = 0.9f;
    Planta planta;
    planta.x = -1.5f; planta.y = -0.8f; planta.theta = grau_para_rad(70.0f);
    if (cen == Cenario::PRESO) planta.preso = true;
    if (cen == Cenario::MOTOR_INVERTIDO) {
        planta.invertido = true;
        planta.theta = std::atan2(alvo_y - planta.y, alvo_x - planta.x);   // já alinhado
    }
    if (cen == Cenario::PARADO_ESTAVEL) {             // já no alvo, visão quase sem ruído
        planta.x = alvo_x - 0.11f; planta.y = alvo_y; planta.theta = 0.0f;
    }

    std::mt19937 rng(3);
    const float sigma_pos = (cen == Cenario::PARADO_ESTAVEL) ? 0.001f : 0.003f;
    std::normal_distribution<float> ruido_pos(0.0f, sigma_pos);
    std::normal_distribution<float> ruido_ang(0.0f, grau_para_rad(1.5f));

    const float dt = 0.005f, periodo_visao = 1.0f / 60.0f, latencia = 0.040f;
    const float duracao = (cen == Cenario::PARADO_ESTAVEL) ? 30.0f : 8.0f;
    std::deque<Medida> fila;
    float t_visao = 0.0f;
    bool congelada = false;
    Medida congelada_em{};
    float x_congelou = 0.0f, y_congelou = 0.0f;
    Resultado r;

    for (int k = 0; k * dt < duracao; ++k) {
        const float t = k * dt;

        // Gatilho do travamento: no meio do movimento que se quer testar
        if (!congelada && !fila.empty()) {
            const bool andando = cen == Cenario::CONGELA_ANDANDO && planta.v > 1.0f;
            const bool girando = cen == Cenario::CONGELA_GIRANDO && std::fabs(planta.w) > 2.0f;
            if (andando || girando) {
                congelada = true;
                congelada_em = fila.back();
                r.t_congelou = t;
                x_congelou = planta.x; y_congelou = planta.y;
            }
        }

        t_visao += dt;
        if (t_visao >= periodo_visao) {
            t_visao -= periodo_visao;
            Medida m{planta.x + ruido_pos(rng), planta.y + ruido_pos(rng),
                     planta.theta + ruido_ang(rng), t};
            if (cen == Cenario::PARADO_ESTAVEL) {     // visão que arredonda para mm
                m.x = std::round(m.x * 1000.0f) / 1000.0f;
                m.y = std::round(m.y * 1000.0f) / 1000.0f;
                m.theta = std::round(m.theta * 1000.0f) / 1000.0f;
            }
            if (congelada) m = Medida{congelada_em.x, congelada_em.y, congelada_em.theta, t};
            fila.push_back(m);
        }
        while (!fila.empty() && t - fila.front().t >= latencia) {
            const Medida m = fila.front();
            fila.pop_front();
            entrada.nova_medida(m.x, m.y, m.theta);
        }
        entrada.avancar(dt);

        const PoseEstimada& p = entrada.pose();       // mesmo corte da ponte_serial
        if (entrada.visao_congelada()) {
            robo.perder_visao();
        } else if (p.valida && p.idade_s <= ctrl.timeout_visao_s) {
            robo.atualizar_visao(p.x, p.y, p.theta, alvo_x, alvo_y);
        }
        const ComandoRodas c = robo.tomar_decisao(dt);
        planta.passo(c, fis, dt);

        // Depois do problema: quando parou, e se voltou a andar depois de parar
        const bool problema = congelada || robo.falhas_movimento() > 0;
        const bool parado = std::fabs(planta.v) < 0.05f && std::fabs(planta.w) < 0.2f;
        if (problema && r.t_parou < 0.0f && parado) r.t_parou = t;
        if (r.t_parou >= 0.0f && congelada && t > r.t_parou + 0.3f) {   // depois de assentar
            const float vel = std::fabs(planta.v) + std::fabs(planta.w) * fis.dist_roda_tras_m;
            r.v_max_depois = (vel > r.v_max_depois) ? vel : r.v_max_depois;
        }
    }

    if (congelada) r.andou_depois_m = std::hypot(planta.x - x_congelou, planta.y - y_congelou);
    r.congelados = entrada.medidas_congeladas();
    r.falhas = robo.falhas_movimento();
    r.dist_final = std::hypot(alvo_x - planta.x, alvo_y - planta.y);

    switch (cen) {
    case Cenario::NORMAL:
        r.passou = r.falhas == 0 && r.congelados == 0 && std::fabs(r.dist_final - 0.11f) < 0.05f;
        r.motivo = "chega no alvo sem falso alarme";
        break;
    case Cenario::CONGELA_ANDANDO:
    case Cenario::CONGELA_GIRANDO:
        // Para em até 1 s, não anda mais de 1 m e não religa sozinho com a visão travada
        r.passou = r.t_congelou >= 0.0f && r.t_parou >= 0.0f && r.t_parou - r.t_congelou < 1.0f &&
                   r.andou_depois_m < 1.0f && r.v_max_depois < 0.05f;
        r.motivo = "para em < 1 s e fica parado";
        break;
    case Cenario::PRESO:
    case Cenario::MOTOR_INVERTIDO:
        r.passou = r.falhas >= 1 && r.t_parou >= 0.0f;
        r.motivo = "detecta e para";
        break;
    case Cenario::PARADO_ESTAVEL:
        r.passou = r.congelados == 0 && r.falhas == 0;
        r.motivo = "visão arredondada em mm não é confundida com travada";
        break;
    }
    return r;
}

}  // namespace

int main() {
    struct { Cenario c; const char* nome; } casos[] = {
        {Cenario::NORMAL,          "normal"},
        {Cenario::CONGELA_ANDANDO, "visao trava andando"},
        {Cenario::CONGELA_GIRANDO, "visao trava girando"},
        {Cenario::PRESO,           "robo preso"},
        {Cenario::MOTOR_INVERTIDO, "motor invertido"},
        {Cenario::PARADO_ESTAVEL,  "parado, visao em mm"},
    };

    std::printf("%-22s %-6s %10s %10s %10s %6s %6s  %s\n", "cenario", "", "parou em",
                "andou", "religou", "repet", "falhas", "criterio");
    int falhou = 0;
    for (const auto& caso : casos) {
        const Resultado r = rodar(caso.c);
        char parou[16] = "-", andou[16] = "-", religou[16] = "-";
        if (r.t_parou >= 0.0f) {
            const float t0 = (r.t_congelou >= 0.0f) ? r.t_congelou : 0.0f;
            std::snprintf(parou, sizeof(parou), "%.2f s", r.t_parou - t0);
        }
        if (r.t_congelou >= 0.0f) {
            std::snprintf(andou, sizeof(andou), "%.2f m", r.andou_depois_m);
            std::snprintf(religou, sizeof(religou), "%.2f m/s", r.v_max_depois);
        }
        std::printf("%-22s %-6s %10s %10s %10s %6d %6d  %s\n", caso.nome,
                    r.passou ? "OK" : "FALHOU", parou, andou, religou, r.congelados, r.falhas,
                    r.motivo);
        if (!r.passou) ++falhou;
    }
    std::printf("\n%s\n", falhou ? "HA CENARIOS FALHANDO" : "todos os cenarios passaram");
    return falhou ? 1 : 0;
}
