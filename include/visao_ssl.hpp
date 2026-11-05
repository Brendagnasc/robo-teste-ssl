/**
 * visao_ssl.hpp
 *
 * Entrada da visão no formato da liga: SSL_WrapperPacket (protobuf) em multicast, que é
 * o que sai do SSL-Vision, do grSim e do ER-Force. Com isso a ponte roda com qualquer
 * um deles, ou com a visão do outro grupo se ela mandar nesse formato.
 *
 * O que muda em relação à entrada em texto (pose ...):
 *   - posição em milímetros e ângulo em radianos, origem no centro do campo;
 *   - cada câmera manda o seu pacote: o robô e a bola podem vir em pacotes
 *     diferentes, e um pacote pode não ter nenhum dos dois;
 *   - o pacote traz t_capture e t_sent. A diferença é o tempo de processamento da
 *     visão, medido no relógio do PC da visão, então vale mesmo com os relógios dos
 *     dois PCs fora de sincronia. Exposição da câmera e rede não aparecem no pacote:
 *     essa parte continua vindo do latencia_s do ConfigEntrada (--latencia).
 *
 * O header não inclui o protobuf, para a ponte não depender dele direto.
 */
#pragma once

namespace ssl {

/** Qual robô é o nosso e o que aceitar. */
struct ConfigSSL {
    bool  time_azul     = true;
    int   id_robo       = 0;
    float confianca_min = 0.3f;   // deteções abaixo disso são ignoradas (robô e bola)
};

/** O que um pacote trouxe, nas unidades da visão (mm, rad). */
struct LeituraSSL {
    int   camera = -1;
    float processamento_s = 0.0f;   // t_sent - t_capture

    bool  tem_robo = false;
    float x = 0.0f, y = 0.0f, theta = 0.0f;
    float confianca_robo = 0.0f;

    bool  tem_bola = false;
    float bola_x = 0.0f, bola_y = 0.0f;
    float confianca_bola = 0.0f;
};

/**
 * Lê um SSL_WrapperPacket serializado. Devolve false se os bytes não formam um pacote
 * ou se ele não tem deteção (pacote só de geometria, por exemplo).
 */
bool ler_pacote_ssl(const void* dados, int tamanho, const ConfigSSL& cfg, LeituraSSL& saida);

/**
 * Socket UDP não bloqueante no grupo multicast da visão (padrão da liga: 224.5.23.2:10020).
 * interface: IP da placa de rede ligada à visão; "0.0.0.0" deixa o sistema escolher.
 * Devolve -1 se falhar.
 */
int abrir_multicast(const char* grupo, int porta, const char* interface = "0.0.0.0");

}  // namespace ssl
