/**
 * visao_ssl.cpp
 * Leitura do SSL_WrapperPacket e socket multicast. Ver visao_ssl.hpp.
 */
#include "visao_ssl.hpp"

#include "messages_robocup_ssl_wrapper.pb.h"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

namespace ssl {

bool ler_pacote_ssl(const void* dados, int tamanho, const ConfigSSL& cfg, LeituraSSL& saida) {
    saida = LeituraSSL{};
    // ParsePartial: o ParseFromArray escreve no stderr a cada pacote incompleto, o que a
    // 60 Hz enche o log da ponte. Só a deteção precisa estar completa; a geometria não é usada.
    SSL_WrapperPacket pacote;
    if (!pacote.ParsePartialFromArray(dados, tamanho) || !pacote.has_detection() ||
        !pacote.detection().IsInitialized()) {
        return false;
    }
    const SSL_DetectionFrame& det = pacote.detection();
    saida.camera = (int)det.camera_id();

    // No grSim t_sent == t_capture e o processamento fica zero. Valor negativo ou
    // grande demais (relógio ajustado no meio do frame) não é confiável: ignora.
    const double proc = det.t_sent() - det.t_capture();
    saida.processamento_s = (proc >= 0.0 && proc < 0.5) ? (float)proc : 0.0f;

    // O mesmo id pode aparecer duas vezes no frame (padrão confundido): fica o mais confiável
    const auto& robos = cfg.time_azul ? det.robots_blue() : det.robots_yellow();
    for (const SSL_DetectionRobot& r : robos) {
        if (!r.has_robot_id() || (int)r.robot_id() != cfg.id_robo) continue;
        if (!r.has_orientation()) continue;   // sem ângulo o controle não tem como alinhar
        if (r.confidence() < cfg.confianca_min) continue;
        if (saida.tem_robo && r.confidence() <= saida.confianca_robo) continue;
        saida.tem_robo = true;
        saida.x = r.x();
        saida.y = r.y();
        saida.theta = r.orientation();
        saida.confianca_robo = r.confidence();
    }

    for (const SSL_DetectionBall& b : det.balls()) {
        if (b.confidence() < cfg.confianca_min) continue;
        if (saida.tem_bola && b.confidence() <= saida.confianca_bola) continue;
        saida.tem_bola = true;
        saida.bola_x = b.x();
        saida.bola_y = b.y();
        saida.confianca_bola = b.confidence();
    }
    return true;
}

int abrir_multicast(const char* grupo, int porta, const char* interface) {
    int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) return -1;

    // Outros programas ouvem a mesma porta (ssl_vision do ssl-robot, visualizador)
    int sim = 1;
    ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &sim, sizeof(sim));

    sockaddr_in end{};
    end.sin_family = AF_INET;
    end.sin_addr.s_addr = htonl(INADDR_ANY);
    end.sin_port = htons((uint16_t)porta);
    if (::bind(fd, (sockaddr*)&end, sizeof(end)) < 0) { ::close(fd); return -1; }

    ip_mreq grupo_mc{};
    if (::inet_pton(AF_INET, grupo, &grupo_mc.imr_multiaddr) != 1 ||
        ::inet_pton(AF_INET, interface, &grupo_mc.imr_interface) != 1 ||
        ::setsockopt(fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &grupo_mc, sizeof(grupo_mc)) < 0) {
        ::close(fd);
        return -1;
    }

    const int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    return fd;
}

}  // namespace ssl
