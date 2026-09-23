/**
 * entrada_visao.cpp
 * Filtro alfa-beta, compensação de latência e rejeição de deteção errada.
 * Ver entrada_visao.hpp para o raciocínio de cada etapa.
 */
#include "entrada_visao.hpp"
#include "ssl_controle.hpp"

#include <cmath>

namespace ssl {

EntradaVisao::EntradaVisao(const ConfigEntrada& cfg) : cfg_(cfg) {}

void EntradaVisao::reiniciar() {
    pose_ = PoseEstimada{};
    iniciado_ = false;
    saltos_seguidos_ = 0;
    t_desde_medida_ = 0.0f;
}

/** Converte as unidades e o referencial da visão para os do controle. */
void EntradaVisao::converter(float x_cru, float y_cru, float theta_cru,
                             float& x, float& y, float& theta) const {
    x = x_cru * cfg_.escala_posicao + cfg_.origem_x;
    y = y_cru * cfg_.escala_posicao * (cfg_.inverter_y ? -1.0f : 1.0f) + cfg_.origem_y;
    theta = cfg_.angulo_em_graus ? grau_para_rad(theta_cru) : theta_cru;
    if (cfg_.inverter_y) theta = -theta;      // espelhar y espelha o sentido de rotação
    theta = normalizar_angulo(theta + cfg_.offset_theta_rad);
}

void EntradaVisao::converter_ponto(float x_cru, float y_cru, float& x, float& y) const {
    float ignorado;
    converter(x_cru, y_cru, 0.0f, x, y, ignorado);
}

bool EntradaVisao::nova_medida(float x_cru, float y_cru, float theta_cru) {
    float zx, zy, zt;
    converter(x_cru, y_cru, theta_cru, zx, zy, zt);

    // Primeiro frame: adota a medida como estimativa inicial, sem velocidade
    if (!iniciado_) {
        pose_ = PoseEstimada{zx, zy, zt, 0.0f, 0.0f, 0.0f, true, 0.0f};
        iniciado_ = true;
        t_desde_medida_ = 0.0f;
        ++aceitas_;
        return true;
    }

    // Intervalo real entre frames, suavizado (a visão nem sempre é regular)
    const float dt_m = (t_desde_medida_ > 1.0e-4f) ? t_desde_medida_ : dt_medida_;
    dt_medida_ = 0.8f * dt_medida_ + 0.2f * dt_m;

    // A medida vale para t - latencia, então a comparação tem que ser feita LÁ.
    // Recua a estimativa pela latência, compara, corrige e projeta de volta para agora.
    // Comparar direto com a estimativa atual geraria um resíduo sempre negativo
    // (da ordem de v * latencia) e o filtro subestimaria a velocidade.
    const float x_med = pose_.x - pose_.vx * cfg_.latencia_s;
    const float y_med = pose_.y - pose_.vy * cfg_.latencia_s;
    const float t_med = normalizar_angulo(pose_.theta - pose_.omega * cfg_.latencia_s);

    const float rx = zx - x_med;
    const float ry = zy - y_med;
    const float rt = normalizar_angulo(zt - t_med);

    // Deteção errada: salto grande demais para a física do robô
    const float salto = std::sqrt(rx * rx + ry * ry);
    if (salto > cfg_.salto_max_m || std::fabs(rt) > cfg_.salto_max_rad) {
        ++saltos_seguidos_;
        ++descartadas_;
        if (saltos_seguidos_ < cfg_.saltos_para_aceitar) {
            return false;                      // ignora e segue com a predição
        }
        // O salto persiste: não era ruído, o robô realmente está lá
        pose_ = PoseEstimada{zx, zy, zt, 0.0f, 0.0f, 0.0f, true, 0.0f};
        saltos_seguidos_ = 0;
        t_desde_medida_ = 0.0f;
        ++aceitas_;
        return true;
    }
    saltos_seguidos_ = 0;

    // Correção alfa-beta no instante da medida: posição puxa pelo resíduo,
    // velocidade puxa pelo resíduo dividido pelo intervalo entre frames
    const float x_corr = x_med + cfg_.alfa * rx;
    const float y_corr = y_med + cfg_.alfa * ry;
    const float t_corr = normalizar_angulo(t_med + cfg_.alfa_angulo * rt);
    pose_.vx += cfg_.beta * rx / dt_medida_;
    pose_.vy += cfg_.beta * ry / dt_medida_;
    pose_.omega += cfg_.beta_angulo * rt / dt_medida_;

    // Projeta de volta para o instante atual, já com a velocidade corrigida
    pose_.x = x_corr + pose_.vx * cfg_.latencia_s;
    pose_.y = y_corr + pose_.vy * cfg_.latencia_s;
    pose_.theta = normalizar_angulo(t_corr + pose_.omega * cfg_.latencia_s);

    pose_.valida = true;
    pose_.idade_s = 0.0f;
    t_desde_medida_ = 0.0f;
    ++aceitas_;
    return true;
}

void EntradaVisao::avancar(float dt) {
    if (!iniciado_) return;
    t_desde_medida_ += dt;
    pose_.idade_s += dt;

    // Entre frames, a estimativa anda sozinha com a velocidade estimada
    pose_.x += pose_.vx * dt;
    pose_.y += pose_.vy * dt;
    pose_.theta = normalizar_angulo(pose_.theta + pose_.omega * dt);

    // Sem frame há tempo demais: a predição já não vale nada
    if (pose_.idade_s > cfg_.timeout_s) {
        pose_.valida = false;
        pose_.vx = pose_.vy = pose_.omega = 0.0f;
    }
}

}  // namespace ssl
