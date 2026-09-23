/**
 * ponte_serial.cpp
 *
 * Fecha o laço: recebe a pose da visão (UDP, vinda do visao_sala.py), roda o
 * ControladorSSL e manda as velocidades das rodas pela USB serial para o Arduino/STM32.
 *
 *   visao_sala.py  --UDP-->  ponte_serial  --USB serial-->  firmware_robo.ino
 *
 * Por que a ponte é C++ e não Python: a lógica de controle (física, rampas, FSM) fica
 * em um único lugar, o mesmo código que vai para o robô da SSL depois. A visão é
 * Python porque OpenCV em Python é mais rápido de iterar.
 *
 * Protocolo serial (texto, fácil de depurar com o monitor serial):
 *     #V,<esq>,<dir>,<tras>,<me>,<md>,<mt>*<checksum>\n
 *   velocidades em mm/s (inteiro), modos em A (ativo), I (inércia), F (freio),
 *   checksum = XOR de todos os bytes entre '#' e '*', em hexadecimal.
 *
 * Uso:
 *     ./ponte_serial --serial /dev/ttyACM0 --baud 115200 --porta-udp 5005
 *     ./ponte_serial --serial /dev/ttyACM0 --dist-parada 0.10
 *
 * Observação para WSL: o Linux do WSL não enxerga /dev/ttyACM0 direto, é preciso
 * anexar o dispositivo com usbipd-win. Rodando no Windows ou em Linux nativo, é direto.
 */

#include "entrada_visao.hpp"
#include "ssl_controle.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

using namespace ssl;

namespace {

std::atomic<bool> rodando{true};
void tratar_sinal(int) { rodando = false; }

/* ------------------------------------------------------------------ serial */

speed_t traduzir_baud(int baud) {
    switch (baud) {
        case 9600:   return B9600;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        default:     return B115200;
    }
}

/** Abre a porta em modo raw: sem eco, sem tradução de caracteres, sem bloquear. */
int abrir_serial(const char* caminho, int baud) {
    int fd = ::open(caminho, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return -1;

    termios tty{};
    if (tcgetattr(fd, &tty) != 0) { ::close(fd); return -1; }
    cfmakeraw(&tty);
    cfsetispeed(&tty, traduzir_baud(baud));
    cfsetospeed(&tty, traduzir_baud(baud));
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CRTSCTS;          // sem controle de fluxo por hardware
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;
    if (tcsetattr(fd, TCSANOW, &tty) != 0) { ::close(fd); return -1; }
    tcflush(fd, TCIOFLUSH);
    return fd;
}

char letra_modo(ModoMotor m) {
    switch (m) {
        case ModoMotor::ATIVO:   return 'A';
        case ModoMotor::FREIO:   return 'F';
        case ModoMotor::INERCIA: return 'I';
    }
    return 'I';
}

/** Monta o quadro com checksum XOR e escreve na serial. */
bool enviar_quadro(int fd, const ComandoRodas& c) {
    char corpo[96];
    const int n = std::snprintf(corpo, sizeof(corpo), "V,%d,%d,%d,%c,%c,%c",
                                (int)std::lround(c.v[RODA_ESQ] * 1000.0f),
                                (int)std::lround(c.v[RODA_DIR] * 1000.0f),
                                (int)std::lround(c.v[RODA_TRAS] * 1000.0f),
                                letra_modo(c.modo[RODA_ESQ]),
                                letra_modo(c.modo[RODA_DIR]),
                                letra_modo(c.modo[RODA_TRAS]));
    unsigned char chk = 0;
    for (int i = 0; i < n; ++i) chk ^= (unsigned char)corpo[i];

    char quadro[128];
    const int m = std::snprintf(quadro, sizeof(quadro), "#%s*%02X\n", corpo, chk);
    return ::write(fd, quadro, (size_t)m) == m;
}

/* --------------------------------------------------------------------- UDP */

int abrir_udp(int porta) {
    int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) return -1;
    sockaddr_in end{};
    end.sin_family = AF_INET;
    end.sin_addr.s_addr = htonl(INADDR_ANY);
    end.sin_port = htons((uint16_t)porta);
    if (::bind(fd, (sockaddr*)&end, sizeof(end)) < 0) { ::close(fd); return -1; }
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    return fd;
}

struct Medida {
    float x, y, theta, ax, ay;
    bool nova = false;
};

/** Esvazia a fila e fica com o pacote mais recente: pose velha só atrasa o controle. */
Medida ler_udp(int fd) {
    Medida m;
    char buf[256];
    while (true) {
        const ssize_t n = ::recv(fd, buf, sizeof(buf) - 1, 0);
        if (n <= 0) break;
        buf[n] = '\0';
        float x, y, th, ax, ay, t;
        if (std::sscanf(buf, "pose %f %f %f %f %f %f", &x, &y, &th, &ax, &ay, &t) == 6) {
            m = Medida{x, y, th, ax, ay, true};
        }
    }
    return m;
}

}  // namespace

int main(int argc, char** argv) {
    std::string caminho_serial = "/dev/ttyACM0";
    int baud = 115200, porta_udp = 5005, hz_controle = 200, hz_serial = 100;
    float dist_parada = 0.0f;
    bool verboso = true;

    // Contrato com o grupo da visão (ver INTERFACE_VISAO.md)
    ConfigEntrada cfg;

    for (int i = 1; i < argc - 1; ++i) {
        const std::string a = argv[i];
        if (a == "--serial")            caminho_serial = argv[++i];
        else if (a == "--baud")         baud = std::atoi(argv[++i]);
        else if (a == "--porta-udp")    porta_udp = std::atoi(argv[++i]);
        else if (a == "--hz")           hz_controle = std::atoi(argv[++i]);
        else if (a == "--hz-serial")    hz_serial = std::atoi(argv[++i]);
        else if (a == "--dist-parada")  dist_parada = (float)std::atof(argv[++i]);
        else if (a == "--milimetros")   { cfg.escala_posicao = 0.001f; --i; }
        else if (a == "--graus")        { cfg.angulo_em_graus = true; --i; }
        else if (a == "--inverter-y")   { cfg.inverter_y = true; --i; }
        else if (a == "--latencia")     cfg.latencia_s = (float)std::atof(argv[++i]);
        else if (a == "--silencioso")   { verboso = false; --i; }
    }

    // Parâmetros do robô de teste: ajuste massa, bitola e rodas aos valores medidos
    ParametrosFisicos fis;
    ParametrosControle ctrl;
    ctrl.dist_parada_m = dist_parada;
    ControladorSSL robo(fis, ctrl);
    EntradaVisao entrada(cfg);

    if (!robo.limites().valido) {
        std::fprintf(stderr, "Limites físicos inválidos: confira offset_cg_m e dist_roda_tras_m\n");
        return 1;
    }

    const int fd_serial = abrir_serial(caminho_serial.c_str(), baud);
    if (fd_serial < 0) {
        std::fprintf(stderr, "Não abri a serial %s\n", caminho_serial.c_str());
        return 1;
    }
    const int fd_udp = abrir_udp(porta_udp);
    if (fd_udp < 0) { std::fprintf(stderr, "Não abri o UDP %d\n", porta_udp); return 1; }

    std::signal(SIGINT, tratar_sinal);
    std::printf("ponte ativa: UDP %d -> %s @ %d bps | v_max %.2f m/s | a %.2f/%.2f m/s^2\n",
                porta_udp, caminho_serial.c_str(), baud,
                robo.limites().v_max, robo.limites().a_acel_max, robo.limites().a_desac_max);
    std::this_thread::sleep_for(std::chrono::seconds(2));  // Arduino reinicia ao abrir a porta

    const auto periodo = std::chrono::microseconds(1000000 / hz_controle);
    const float dt = 1.0f / (float)hz_controle;
    const int div_serial = (hz_controle > hz_serial) ? hz_controle / hz_serial : 1;

    auto proximo = std::chrono::steady_clock::now();
    float alvo_x = 0.0f, alvo_y = 0.0f;
    bool tem_alvo = false;
    int ciclo = 0;
    float t_log = 0.0f;

    while (rodando) {
        const Medida m = ler_udp(fd_udp);
        if (m.nova) {
            entrada.nova_medida(m.x, m.y, m.theta);
            entrada.converter_ponto(m.ax, m.ay, alvo_x, alvo_y);
            tem_alvo = true;
        }
        entrada.avancar(dt);

        // O controle recebe a pose estimada para AGORA, não a medida atrasada
        const PoseEstimada& p = entrada.pose();
        if (p.valida && tem_alvo) robo.atualizar_visao(p.x, p.y, p.theta, alvo_x, alvo_y);

        const ComandoRodas c = robo.tomar_decisao(dt);
        if (ciclo % div_serial == 0) {
            if (!enviar_quadro(fd_serial, c)) std::fprintf(stderr, "falha ao escrever na serial\n");
        }

        t_log += dt;
        if (verboso && t_log >= 0.5f) {
            t_log = 0.0f;
            std::printf("%-12s d=%.3f m  erro=%+.1f deg  v=%.2f m/s  visao: %d ok / %d ruins, "
                        "idade %.0f ms  rodas: %.2f %.2f %.2f\n",
                        nome_estado(robo.estado()), robo.distancia(),
                        rad_para_grau(robo.erro_angulo()), robo.velocidade_linear(),
                        entrada.medidas_aceitas(), entrada.medidas_descartadas(),
                        entrada.pose().idade_s * 1000.0f,
                        c.v[RODA_ESQ], c.v[RODA_DIR], c.v[RODA_TRAS]);
            std::fflush(stdout);
        }

        ++ciclo;
        proximo += periodo;
        std::this_thread::sleep_until(proximo);
    }

    // Saída limpa: manda parada com freio antes de fechar
    ComandoRodas parada;
    parada.modo[RODA_ESQ] = parada.modo[RODA_DIR] = ModoMotor::FREIO;
    for (int i = 0; i < 5; ++i) {
        enviar_quadro(fd_serial, parada);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ::close(fd_serial);
    ::close(fd_udp);
    std::printf("\nponte encerrada, robô parado\n");
    return 0;
}
