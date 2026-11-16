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
 *     ./ponte_serial --serial /dev/rfcomm0 --bluetooth          (HC-05 pareado)
 *     ./ponte_serial --serial /dev/ttyACM0 --entrada ssl --time azul --id 0
 *     ./ponte_serial --serial /dev/ttyACM0 --v-roda-max 0.9     (V_MAX_MM_S de 900 mm/s)
 *
 * Velocidade da roda: o firmware satura o PWM em V_MAX_MM_S, então nenhuma roda passa
 * disso. Sem --v-roda-max o controle calcula o teto pelo rpm do motor e pela redução, e
 * pode planejar velocidades que a roda não entrega. Passe o mesmo valor do firmware, em
 * m/s; o controle usa o fator de segurança em cima dele (80%), o que sobra para a
 * correção de rumo sem saturar a roda.
 *
 * Entrada da visão: "texto" (padrão) é a linha "pose ..." do INTERFACE_VISAO.md, em UDP
 * 5005. "ssl" é o SSL_WrapperPacket no multicast da liga (224.5.23.2:10020), o mesmo do
 * SSL-Vision, do grSim e do ER-Force. Nesse modo o alvo é a bola, a posição vem em mm
 * e a latência de cada frame é o --latencia mais o processamento que o pacote informa
 * (ver visao_ssl.hpp).
 *
 * Bluetooth: a porta rfcomm se comporta como uma serial comum, então nada muda no
 * código. O que muda é a taxa de envio. O Bluetooth clássico trabalha em janelas de
 * 7,5 ms e tem jitter, e o HC-05 via SoftwareSerial roda a 38400 bps, então mandar
 * 100 quadros por segundo só enche fila e faz o robô responder ao passado. A opção
 * --bluetooth baixa o envio para 50 Hz. O controle continua a 200 Hz internamente:
 * só o envio é decimado.
 *
 * Observação para WSL: o Linux do WSL não enxerga /dev/ttyACM0 direto, é preciso
 * anexar o dispositivo com usbipd-win. Rodando no Windows ou em Linux nativo, é direto.
 */

#include "entrada_visao.hpp"
#include "ssl_controle.hpp"
#ifdef COM_VISAO_SSL
#include "visao_ssl.hpp"
#endif

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

/** O que chegou da visão neste ciclo. Robô e alvo podem chegar separados (modo ssl). */
struct Medida {
    bool  robo = false;
    float x = 0.0f, y = 0.0f, theta = 0.0f;
    float latencia = -1.0f;   // negativo: usa a do ConfigEntrada
    bool  alvo = false;
    float ax = 0.0f, ay = 0.0f;
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
            m = Medida{};
            m.robo = m.alvo = true;
            m.x = x; m.y = y; m.theta = th;
            m.ax = ax; m.ay = ay;
        }
    }
    return m;
}

#ifdef COM_VISAO_SSL
/**
 * Mesma ideia no modo ssl, mas cada câmera manda o seu pacote: o robô fica com a
 * deteção mais recente que o mostrou e a bola com a mais recente que a mostrou,
 * mesmo que venham de câmeras diferentes.
 */
Medida ler_ssl(int fd, const ConfigSSL& cfg, float latencia_base) {
    Medida m;
    static char buf[65536];
    while (true) {
        const ssize_t n = ::recv(fd, buf, sizeof(buf), 0);
        if (n <= 0) break;
        LeituraSSL l;
        if (!ler_pacote_ssl(buf, (int)n, cfg, l)) continue;
        if (l.tem_robo) {
            m.robo = true;
            m.x = l.x; m.y = l.y; m.theta = l.theta;
            m.latencia = latencia_base + l.processamento_s;
        }
        if (l.tem_bola) {
            m.alvo = true;
            m.ax = l.bola_x; m.ay = l.bola_y;
        }
    }
    return m;
}
#endif

}  // namespace

int main(int argc, char** argv) {
    std::string caminho_serial = "/dev/ttyACM0";
    int baud = 115200, porta_udp = 5005, hz_controle = 200, hz_serial = 100;
    float dist_parada = 0.0f, v_roda_max = 0.0f;
    bool verboso = true, bluetooth = false;

    // Contrato com o grupo da visão (ver INTERFACE_VISAO.md)
    ConfigEntrada cfg;

    // Modo ssl: qual robô seguir e de onde vem o multicast
    std::string entrada_tipo = "texto", grupo = "224.5.23.2", interface = "0.0.0.0";
    std::string time_cor = "azul";
    int id_robo = 0;
    float confianca_min = 0.3f;

    bool hz_serial_explicito = false, porta_explicita = false;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];

        // Flags sem valor
        if (a == "--milimetros")      { cfg.escala_posicao = 0.001f; continue; }
        if (a == "--graus")           { cfg.angulo_em_graus = true; continue; }
        if (a == "--inverter-y")      { cfg.inverter_y = true; continue; }
        if (a == "--bluetooth")       { bluetooth = true; continue; }
        if (a == "--silencioso")      { verboso = false; continue; }

        // Opções com valor
        const bool conhecida = a == "--serial" || a == "--baud" || a == "--porta-udp" ||
                               a == "--hz" || a == "--hz-serial" || a == "--dist-parada" ||
                               a == "--latencia" || a == "--entrada" || a == "--grupo" ||
                               a == "--interface" || a == "--time" || a == "--id" ||
                               a == "--confianca-min" || a == "--v-roda-max";
        if (!conhecida) {
            std::fprintf(stderr, "Opção desconhecida: %s\n", a.c_str());
            return 1;
        }
        if (i + 1 >= argc) {
            std::fprintf(stderr, "Falta o valor de %s\n", a.c_str());
            return 1;
        }
        const char* valor = argv[++i];
        if (a == "--serial")            caminho_serial = valor;
        else if (a == "--baud")         baud = std::atoi(valor);
        else if (a == "--porta-udp")    { porta_udp = std::atoi(valor); porta_explicita = true; }
        else if (a == "--hz")           hz_controle = std::atoi(valor);
        else if (a == "--hz-serial")    { hz_serial = std::atoi(valor); hz_serial_explicito = true; }
        else if (a == "--dist-parada")  dist_parada = (float)std::atof(valor);
        else if (a == "--latencia")     cfg.latencia_s = (float)std::atof(valor);
        else if (a == "--entrada")      entrada_tipo = valor;
        else if (a == "--grupo")        grupo = valor;
        else if (a == "--interface")    interface = valor;
        else if (a == "--time")         time_cor = valor;
        else if (a == "--id")           id_robo = std::atoi(valor);
        else if (a == "--confianca-min") confianca_min = (float)std::atof(valor);
        else if (a == "--v-roda-max")   v_roda_max = (float)std::atof(valor);
    }
    if (entrada_tipo != "texto" && entrada_tipo != "ssl") {
        std::fprintf(stderr, "--entrada precisa ser texto ou ssl\n");
        return 1;
    }
    if (time_cor != "azul" && time_cor != "amarelo") {
        std::fprintf(stderr, "--time precisa ser azul ou amarelo\n");
        return 1;
    }
    const bool entrada_ssl = entrada_tipo == "ssl";
#ifndef COM_VISAO_SSL
    if (entrada_ssl) {
        std::fprintf(stderr, "ponte compilada sem protobuf: --entrada ssl indisponível\n");
        return 1;
    }
#else
    ConfigSSL cfg_ssl;
    cfg_ssl.time_azul = time_cor == "azul";
    cfg_ssl.id_robo = id_robo;
    cfg_ssl.confianca_min = confianca_min;
#endif
    if (entrada_ssl) {
        cfg.escala_posicao = 0.001f;              // SSL-Vision manda milímetros
        if (!porta_explicita) porta_udp = 10020;
    }
    // Aplicado depois do laço para não depender da ordem: --hz-serial explícito vence.
    // A taxa do lado do PC não importa no rfcomm; 38400 é só o que aparece no log.
    if (bluetooth) {
        if (!hz_serial_explicito) hz_serial = 50;
        baud = 38400;
    }
    if (hz_controle <= 0 || hz_serial <= 0) {
        std::fprintf(stderr, "--hz e --hz-serial precisam ser maiores que zero\n");
        return 1;
    }
    if (v_roda_max < 0.0f) {
        std::fprintf(stderr, "--v-roda-max precisa ser positivo (m/s)\n");
        return 1;
    }

    // Parâmetros do robô de teste: ajuste massa, bitola e rodas aos valores medidos
    ParametrosFisicos fis;
    fis.v_roda_max_medida = v_roda_max;
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
#ifdef COM_VISAO_SSL
    const int fd_udp = entrada_ssl ? abrir_multicast(grupo.c_str(), porta_udp, interface.c_str())
                                   : abrir_udp(porta_udp);
#else
    const int fd_udp = abrir_udp(porta_udp);
#endif
    if (fd_udp < 0) { std::fprintf(stderr, "Não abri o UDP %d\n", porta_udp); return 1; }

    std::signal(SIGINT, tratar_sinal);
    std::signal(SIGTERM, tratar_sinal);   // kill / terminate também passam pela parada com freio
    if (entrada_ssl) {
        std::printf("visao: SSL multicast %s:%d, robo %s %d, confianca >= %.2f\n",
                    grupo.c_str(), porta_udp, time_cor.c_str(), id_robo, confianca_min);
    }
    if (v_roda_max > 0.0f) {
        std::printf("roda limitada a %.2f m/s (V_MAX_MM_S do firmware)\n", v_roda_max);
    }
    std::printf("ponte ativa: UDP %d -> %s @ %d bps (%s, envio a %d Hz)\n"
                "v_max %.2f m/s | a %.2f/%.2f m/s^2\n",
                porta_udp, caminho_serial.c_str(), baud,
                bluetooth ? "bluetooth" : "cabo", hz_serial,
                robo.limites().v_max, robo.limites().a_acel_max, robo.limites().a_desac_max);
    // Cabo: o Arduino reinicia ao abrir a porta. Bluetooth: o enlace leva um instante.
    std::this_thread::sleep_for(std::chrono::seconds(2));

    const auto periodo = std::chrono::microseconds(1000000 / hz_controle);
    const float dt = 1.0f / (float)hz_controle;
    const int div_serial = (hz_controle > hz_serial) ? hz_controle / hz_serial : 1;

    auto proximo = std::chrono::steady_clock::now();
    float alvo_x = 0.0f, alvo_y = 0.0f;
    bool tem_alvo = false;
    int ciclo = 0;
    float t_log = 0.0f;
    float lat_usada = cfg.latencia_s;   // só para o log

    while (rodando) {
#ifdef COM_VISAO_SSL
        const Medida m = entrada_ssl ? ler_ssl(fd_udp, cfg_ssl, cfg.latencia_s) : ler_udp(fd_udp);
#else
        const Medida m = ler_udp(fd_udp);
#endif
        if (m.robo) {
            entrada.nova_medida(m.x, m.y, m.theta, m.latencia);
            if (m.latencia >= 0.0f) lat_usada = m.latencia;
        }
        // Modo ssl: sem bola no frame, o alvo continua sendo a última posição vista
        if (m.alvo) {
            entrada.converter_ponto(m.ax, m.ay, alvo_x, alvo_y);
            tem_alvo = true;
        }
        entrada.avancar(dt);

        // O controle recebe a pose estimada para AGORA, não a medida atrasada.
        // Só repassa enquanto o último frame é recente: a predição é chamada a cada
        // ciclo, então sem esse corte o timeout_visao_s do controle nunca dispararia.
        const PoseEstimada& p = entrada.pose();
        if (entrada.visao_congelada()) {
            robo.perder_visao();               // visão repetindo a pose: freia já
        } else if (p.valida && tem_alvo && p.idade_s <= ctrl.timeout_visao_s) {
            robo.atualizar_visao(p.x, p.y, p.theta, alvo_x, alvo_y);
        }

        const ComandoRodas c = robo.tomar_decisao(dt);
        if (ciclo % div_serial == 0) {
            if (!enviar_quadro(fd_serial, c)) std::fprintf(stderr, "falha ao escrever na serial\n");
        }

        t_log += dt;
        if (verboso && t_log >= 0.5f) {
            t_log = 0.0f;
            std::printf("%-12s d=%.3f m  erro=%+.1f deg  v=%.2f m/s  visao: %d ok / %d ruins / "
                        "%d repetidos, idade %.0f ms, lat %.0f ms  rodas: %.2f %.2f %.2f%s%s\n",
                        nome_estado(robo.estado()), robo.distancia(),
                        rad_para_grau(robo.erro_angulo()), robo.velocidade_linear(),
                        entrada.medidas_aceitas(), entrada.medidas_descartadas(),
                        entrada.medidas_congeladas(), entrada.pose().idade_s * 1000.0f,
                        lat_usada * 1000.0f,
                        c.v[RODA_ESQ], c.v[RODA_DIR], c.v[RODA_TRAS],
                        entrada.visao_congelada() ? "  [VISAO TRAVADA]" : "",
                        robo.falha_movimento() ? "  [SEM MOVIMENTO: parado]" : "");
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
