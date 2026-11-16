/**
 * teste_visao_ssl.cpp
 *
 * Confere a leitura do SSL_WrapperPacket (visao_ssl.cpp) com pacotes montados aqui:
 * time e id certos, confiança mínima, deteção duplicada, robô sem ângulo, bola,
 * tempo de processamento (inclusive o do grSim, que é zero) e pacote que não é de deteção.
 *
 * No fim, mede o ganho de passar a latência de cada frame para a EntradaVisao quando
 * o atraso da visão varia de um frame para outro. Devolve 0 se tudo passa.
 */
#include "entrada_visao.hpp"
#include "ssl_controle.hpp"
#include "visao_ssl.hpp"

#include "messages_robocup_ssl_wrapper.pb.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

using namespace ssl;

namespace {

int falhas = 0;

void conferir(bool ok, const char* nome) {
    std::printf("  %-52s %s\n", nome, ok ? "ok" : "FALHOU");
    if (!ok) ++falhas;
}

bool perto(float a, float b, float tol = 1e-4f) { return std::fabs(a - b) <= tol; }

SSL_DetectionFrame* novo_frame(SSL_WrapperPacket& p, int camera, double t_cap, double t_env) {
    SSL_DetectionFrame* f = p.mutable_detection();
    f->set_frame_number(1);
    f->set_camera_id(camera);
    f->set_t_capture(t_cap);
    f->set_t_sent(t_env);
    return f;
}

void por_robo(google::protobuf::RepeatedPtrField<SSL_DetectionRobot>* lista, int id,
              float x, float y, float th, float conf, bool com_angulo = true) {
    SSL_DetectionRobot* r = lista->Add();
    r->set_confidence(conf);
    r->set_robot_id(id);
    r->set_x(x);
    r->set_y(y);
    if (com_angulo) r->set_orientation(th);
    r->set_pixel_x(0);
    r->set_pixel_y(0);
}

void por_bola(SSL_DetectionFrame* f, float x, float y, float conf) {
    SSL_DetectionBall* b = f->add_balls();
    b->set_confidence(conf);
    b->set_x(x);
    b->set_y(y);
    b->set_pixel_x(0);
    b->set_pixel_y(0);
}

bool ler(const SSL_WrapperPacket& p, const ConfigSSL& cfg, LeituraSSL& l) {
    const std::string bytes = p.SerializeAsString();
    return ler_pacote_ssl(bytes.data(), (int)bytes.size(), cfg, l);
}

void testar_leitura() {
    std::printf("leitura do pacote\n");
    ConfigSSL azul0;
    LeituraSSL l;

    {   // Cena com o nosso robô, o amarelo de mesmo id e outro azul
        SSL_WrapperPacket p;
        SSL_DetectionFrame* f = novo_frame(p, 2, 100.000, 100.004);
        por_robo(f->mutable_robots_blue(), 1, 900, 900, 0.0f, 0.9f);
        por_robo(f->mutable_robots_blue(), 0, 1200, -350, 1.2f, 0.8f);
        por_robo(f->mutable_robots_yellow(), 0, -500, 400, -2.0f, 0.95f);
        por_bola(f, 300, 200, 0.9f);

        const bool ok = ler(p, azul0, l);
        conferir(ok && l.tem_robo && perto(l.x, 1200) && perto(l.y, -350) && perto(l.theta, 1.2f),
                 "azul 0: pega o robo certo");
        conferir(l.camera == 2, "guarda o id da camera");
        conferir(perto(l.processamento_s, 0.004f, 1e-5f), "processamento = t_sent - t_capture");
        conferir(l.tem_bola && perto(l.bola_x, 300) && perto(l.bola_y, 200), "bola");

        ConfigSSL amarelo0;
        amarelo0.time_azul = false;
        ler(p, amarelo0, l);
        conferir(l.tem_robo && perto(l.x, -500) && perto(l.theta, -2.0f), "amarelo 0: pega o do outro time");

        ConfigSSL azul3;
        azul3.id_robo = 3;
        ler(p, azul3, l);
        conferir(!l.tem_robo && l.tem_bola, "id ausente: sem robo, bola continua");
    }

    {   // Id duplicado e deteções fracas
        SSL_WrapperPacket p;
        SSL_DetectionFrame* f = novo_frame(p, 0, 5.0, 5.0);
        por_robo(f->mutable_robots_blue(), 0, 3000, 3000, 0.0f, 0.4f);
        por_robo(f->mutable_robots_blue(), 0, 100, 100, 0.5f, 0.9f);
        por_robo(f->mutable_robots_blue(), 0, -900, -900, 0.0f, 0.6f);
        por_bola(f, -2000, 0, 0.5f);
        por_bola(f, 50, 60, 0.85f);
        por_bola(f, 4000, 0, 0.1f);

        ler(p, azul0, l);
        conferir(l.tem_robo && perto(l.x, 100) && perto(l.theta, 0.5f), "id duplicado: fica o mais confiavel");
        conferir(l.tem_bola && perto(l.bola_x, 50), "varias bolas: fica a mais confiavel");
        conferir(l.processamento_s == 0.0f, "grSim (t_sent == t_capture): processamento zero");

        ConfigSSL exigente;
        exigente.confianca_min = 0.95f;
        ler(p, exigente, l);
        conferir(!l.tem_robo && !l.tem_bola, "abaixo da confianca minima: ignora");
    }

    {   // Robô sem ângulo e relógio estranho
        SSL_WrapperPacket p;
        SSL_DetectionFrame* f = novo_frame(p, 1, 10.0, 9.9);
        por_robo(f->mutable_robots_blue(), 0, 10, 10, 0.0f, 0.9f, false);
        ler(p, azul0, l);
        conferir(!l.tem_robo, "robo sem orientacao: ignora");
        conferir(l.processamento_s == 0.0f, "t_sent antes de t_capture: processamento zero");

        f->set_t_sent(12.0);
        ler(p, azul0, l);
        conferir(l.processamento_s == 0.0f, "processamento de 2 s: descarta");
    }

    {   // Pacotes que não são de deteção
        SSL_WrapperPacket so_geometria;
        so_geometria.mutable_geometry()->mutable_field();
        const std::string g = so_geometria.SerializePartialAsString();
        conferir(!ler_pacote_ssl(g.data(), (int)g.size(), azul0, l), "pacote so de geometria: false");

        const char* texto = "pose 1.0 2.0 0.5 3.0 4.0 12.3";
        conferir(!ler_pacote_ssl(texto, (int)std::strlen(texto), azul0, l),
                 "linha de texto no lugar do protobuf: false");

        SSL_WrapperPacket vazio;
        novo_frame(vazio, 0, 1.0, 1.0);
        conferir(ler(vazio, azul0, l) && !l.tem_robo && !l.tem_bola,
                 "frame sem nada: true, sem robo e sem bola");
    }
}

/**
 * Robô em linha reta a 1 m/s, visão a 60 Hz com atraso que varia entre 15 e 65 ms
 * (processamento instável). Compara o erro da pose estimada usando a latência média
 * fixa contra a latência informada em cada frame.
 */
float erro_rms(bool por_frame) {
    ConfigEntrada cfg;
    cfg.latencia_s = 0.040f;          // a média dos atrasos abaixo
    EntradaVisao e(cfg);

    const float v = 1.0f, dt = 0.005f, periodo_visao = 1.0f / 60.0f;
    const float atrasos[] = {0.015f, 0.065f, 0.025f, 0.055f, 0.040f};
    float soma = 0.0f, prox_frame = 0.0f;
    int n = 0, k = 0;
    for (float t = 0.0f; t < 3.0f; t += dt) {
        if (t >= prox_frame) {
            const float atraso = atrasos[k++ % 5];
            const float x_medido = v * (t - atraso);
            e.nova_medida(x_medido, 0.001f * (float)(k % 7), 0.0f, por_frame ? atraso : -1.0f);
            prox_frame += periodo_visao;
        }
        e.avancar(dt);
        if (t > 1.0f) {               // depois do filtro assentar
            const float erro = e.pose().x - v * t;
            soma += erro * erro;
            ++n;
        }
    }
    return std::sqrt(soma / (float)n);
}

void testar_latencia_por_frame() {
    std::printf("\nlatencia por frame (robo a 1 m/s, atraso de 15 a 65 ms)\n");
    const float fixa = erro_rms(false), por_frame = erro_rms(true);
    std::printf("  erro RMS com latencia fixa de 40 ms: %6.1f mm\n", fixa * 1000.0f);
    std::printf("  erro RMS com latencia de cada frame: %6.1f mm\n", por_frame * 1000.0f);
    conferir(por_frame < 0.5f * fixa, "latencia de cada frame reduz o erro pela metade");
}

}  // namespace

int main() {
    GOOGLE_PROTOBUF_VERIFY_VERSION;
    testar_leitura();
    testar_latencia_por_frame();
    std::printf("\n%s\n", falhas ? "HA TESTES FALHANDO" : "todos os testes passaram");
    return falhas ? 1 : 0;
}
