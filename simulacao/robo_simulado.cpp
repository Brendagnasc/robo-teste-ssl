/**
 * robo_simulado.cpp
 *
 * Plugin do Gazebo que faz o papel do hardware do robô de teste (firmware, motores e
 * câmera), para a ponte_serial rodar contra o simulador SEM NENHUMA MUDANÇA:
 *
 *   ponte_serial --serial /tmp/robo_teste_serial --pty--> firmware simulado --torque--> rodas
 *        ^                                                                               |
 *        +----------- UDP 5005 "pose x y theta alvo_x alvo_y t" <---- câmera simulada <---+
 *
 * Firmware: os mesmos quadros do firmware_robo.ino (#V e #P, checksum XOR, linha de até
 *   63 caracteres), com velocidade de até 1 mm/s no modo A virando inércia, saturação
 *   em V_MAX_MM_S e o watchdog pondo tudo em inércia sem quadro válido.
 *
 * Motores: o torque sai daqui a cada passo da física, e não de um JointController. É o
 *   que permite inércia de verdade, com a roda livre: o JointController só segue
 *   velocidade, e pedir 0 rad/s a ele SEGURA a roda, o que no giro freia o robô.
 *     A (ativo):   torque = ganho * (w_ref - w), saturado no torque máximo
 *     F (freio):   o mesmo com w_ref = 0, como o curto entre os terminais
 *     I (inércia): torque zero
 *   Estabilidade: com a roda patinando sobra só a inércia dela, e esse laço explícito
 *   exige ganho * passo / inercia_roda < 2. Com 1,8e-5 kg.m2 e passo de 1 ms, o ganho
 *   precisa ficar abaixo de 0,036. O p_gain de 0,2 do modelo antigo dava 11, e a
 *   simulação explodia assim que a roda patinava.
 *
 * Câmera: a pose do modelo (ponto médio do eixo das rodas laterais) e a do alvo, na
 *   taxa, no atraso e no ruído configurados, com perda de frame e deteção errada
 *   opcionais. Frame perdido não é mandado, como pede o INTERFACE_VISAO.md.
 */

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>

#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <random>
#include <string>

namespace robo_teste {

using namespace gz::sim;

namespace {

// Mesma ordem dos quadros e do ssl_controle.hpp
enum Roda { RODA_ESQ = 0, RODA_DIR = 1, RODA_TRAS = 2, NUM_RODAS = 3 };

double segundos(const std::chrono::steady_clock::duration& d) {
    return std::chrono::duration<double>(d).count();
}

/** Lê um parâmetro de dentro do <plugin> no SDF, com valor padrão. */
template <typename T>
T parametro(const std::shared_ptr<const sdf::Element>& sdf, const char* nome, const T& padrao) {
    return sdf->Get<T>(nome, padrao).first;
}

struct FrameVisao {
    double t_entrega;     // instante (de simulação) em que o frame chega na porta da ponte
    std::string texto;
};

}  // namespace

class RoboSimulado : public System,
                     public ISystemConfigure,
                     public ISystemPreUpdate,
                     public ISystemPostUpdate {
public:
    ~RoboSimulado() override;
    void Configure(const Entity& entidade, const std::shared_ptr<const sdf::Element>& sdf,
                   EntityComponentManager& ecm, EventManager& eventos) override;
    void PreUpdate(const UpdateInfo& info, EntityComponentManager& ecm) override;
    void PostUpdate(const UpdateInfo& info, const EntityComponentManager& ecm) override;

private:
    bool abrir_serial(const std::string& link);
    void ler_serial(double t);
    bool processar(const std::string& linha, double t);
    void definir(int roda, double v_mm_s, char modo);
    double torque(int roda, double w) const;
    void capturar(double t, const EntityComponentManager& ecm);

    Model modelo_;
    std::array<Joint, NUM_RODAS> juntas_;
    bool pronto_ = false;

    // Roda e motor: os mesmos números do ParametrosFisicos e do firmware
    double raio_roda_m_   = 0.027;
    double torque_max_Nm_ = 0.32;   // 0,10 N.m no motor x redução 3,6 x eficiência 0,9
    double ganho_Nms_     = 0.02;   // N.m por rad/s de erro; ver estabilidade no topo
    double v_max_m_s_     = 0.9;    // V_MAX_MM_S: velocidade da roda com PWM 100%
    int    pwm_minimo_    = 45;     // PWM_MINIMO, só para os quadros #P
    double watchdog_s_    = 0.2;    // TIMEOUT_MS com cabo; com Bluetooth é 0,3

    // Comando em vigor, guardado como o firmware guarda
    std::array<double, NUM_RODAS> w_ref_{};          // rad/s
    std::array<char, NUM_RODAS>   modo_{'I', 'I', 'I'};
    double t_ultimo_quadro_ = -1e9;
    bool   em_watchdog_ = true;
    int    quadros_ruins_ = 0;

    // Serial simulada: a ponte abre o link, que aponta para o escravo da pty
    int fd_mestre_ = -1, fd_escravo_ = -1;
    std::string link_serial_, nome_pty_, resto_;

    // Câmera simulada
    int fd_udp_ = -1;
    sockaddr_in destino_{};
    std::string nome_alvo_ = "alvo";
    Entity alvo_ = kNullEntity;
    bool   sem_alvo_avisado_ = false;
    double periodo_s_ = 1.0 / 60.0, latencia_s_ = 0.040;
    double ruido_pos_m_ = 0.002, ruido_ang_rad_ = 0.005;
    double perda_frames_ = 0.0, deteccao_errada_ = 0.0;
    bool   milimetros_ = true;
    double t_proximo_frame_ = 0.0, t_anterior_ = 0.0;
    std::deque<FrameVisao> fila_;
    std::mt19937 rng_{std::random_device{}()};
    std::normal_distribution<double> normal_{0.0, 1.0};
    std::uniform_real_distribution<double> uniforme_{0.0, 1.0};
};

RoboSimulado::~RoboSimulado() {
    // Só apaga o link se ele ainda apontar para a nossa pty: outra simulação pode ter
    // subido depois e criado o dela.
    char destino[256];
    const ssize_t n = ::readlink(link_serial_.c_str(), destino, sizeof(destino) - 1);
    if (n > 0 && nome_pty_ == std::string(destino, (size_t)n)) ::unlink(link_serial_.c_str());
    for (int fd : {fd_escravo_, fd_mestre_, fd_udp_}) {
        if (fd >= 0) ::close(fd);
    }
}

void RoboSimulado::Configure(const Entity& entidade, const std::shared_ptr<const sdf::Element>& sdf,
                             EntityComponentManager& ecm, EventManager&) {
    modelo_ = Model(entidade);
    if (!modelo_.Valid(ecm)) {
        gzerr << "robo_simulado: o plugin tem que ficar dentro do <model> do robô\n";
        return;
    }

    const char* juntas_padrao[NUM_RODAS] = {"junta_esq", "junta_dir", "junta_tras"};
    for (int i = 0; i < NUM_RODAS; ++i) {
        const std::string nome = parametro(sdf, juntas_padrao[i], std::string(juntas_padrao[i]));
        juntas_[i] = Joint(modelo_.JointByName(ecm, nome));
        if (!juntas_[i].Valid(ecm)) {
            gzerr << "robo_simulado: não achei a junta [" << nome << "]\n";
            return;
        }
        juntas_[i].EnableVelocityCheck(ecm, true);   // sem isso o Gazebo não informa a velocidade
    }

    raio_roda_m_   = parametro(sdf, "raio_roda_m", raio_roda_m_);
    torque_max_Nm_ = parametro(sdf, "torque_max_Nm", torque_max_Nm_);
    ganho_Nms_     = parametro(sdf, "ganho_Nms", ganho_Nms_);
    v_max_m_s_     = parametro(sdf, "v_max_m_s", v_max_m_s_);
    pwm_minimo_    = parametro(sdf, "pwm_minimo", pwm_minimo_);
    watchdog_s_    = parametro(sdf, "watchdog_s", watchdog_s_);

    nome_alvo_       = parametro(sdf, "alvo", nome_alvo_);
    periodo_s_       = 1.0 / parametro(sdf, "visao_hz", 60.0);
    latencia_s_      = parametro(sdf, "visao_latencia_s", latencia_s_);
    ruido_pos_m_     = parametro(sdf, "visao_ruido_pos_m", ruido_pos_m_);
    ruido_ang_rad_   = parametro(sdf, "visao_ruido_ang_rad", ruido_ang_rad_);
    perda_frames_    = parametro(sdf, "visao_perda_frames", perda_frames_);
    deteccao_errada_ = parametro(sdf, "visao_deteccao_errada", deteccao_errada_);
    milimetros_      = parametro(sdf, "visao_milimetros", milimetros_);
    const int semente = parametro(sdf, "semente", -1);   // fixe para repetir a mesma corrida
    if (semente >= 0) rng_.seed((unsigned)semente);

    if (!abrir_serial(parametro(sdf, "serial", std::string("/tmp/robo_teste_serial")))) return;

    const std::string host = parametro(sdf, "visao_host", std::string("127.0.0.1"));
    fd_udp_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    destino_.sin_family = AF_INET;
    destino_.sin_port = htons((uint16_t)parametro(sdf, "visao_porta", 5005));
    if (fd_udp_ < 0 || ::inet_pton(AF_INET, host.c_str(), &destino_.sin_addr) != 1) {
        gzerr << "robo_simulado: não abri o UDP para " << host << "\n";
        return;
    }

    pronto_ = true;
    std::printf("robo_simulado: serial em %s (%s) | visão para %s:%d, %.0f Hz, atraso %.0f ms, "
                "ruído %.1f mm e %.2f graus, em %s\n",
                link_serial_.c_str(), nome_pty_.c_str(), host.c_str(), ntohs(destino_.sin_port),
                1.0 / periodo_s_, latencia_s_ * 1000.0, ruido_pos_m_ * 1000.0,
                ruido_ang_rad_ * 180.0 / M_PI, milimetros_ ? "milímetros" : "metros");
    std::fflush(stdout);
}

/**
 * Cria a pty que faz o papel da serial do Arduino e um link de nome fixo para ela,
 * para a ponte não precisar descobrir qual /dev/pts/N saiu desta vez.
 */
bool RoboSimulado::abrir_serial(const std::string& link) {
    fd_mestre_ = ::posix_openpt(O_RDWR | O_NOCTTY);
    if (fd_mestre_ < 0 || ::grantpt(fd_mestre_) != 0 || ::unlockpt(fd_mestre_) != 0) {
        gzerr << "robo_simulado: não consegui criar a pty\n";
        return false;
    }
    nome_pty_ = ::ptsname(fd_mestre_);

    // O plugin segura o escravo aberto: sem ninguém do outro lado, ler o mestre dá EIO.
    // Assim a ponte pode fechar e reabrir a porta à vontade.
    fd_escravo_ = ::open(nome_pty_.c_str(), O_RDWR | O_NOCTTY);
    termios tty{};
    if (fd_escravo_ >= 0 && ::tcgetattr(fd_escravo_, &tty) == 0) {
        ::cfmakeraw(&tty);                 // sem eco: o que a ponte escreve não volta para ela
        ::tcsetattr(fd_escravo_, TCSANOW, &tty);
    }
    ::fcntl(fd_mestre_, F_SETFL, ::fcntl(fd_mestre_, F_GETFL, 0) | O_NONBLOCK);

    link_serial_ = link;
    struct stat st{};
    if (::lstat(link.c_str(), &st) == 0 && S_ISLNK(st.st_mode)) {
        ::unlink(link.c_str());            // link velho de uma simulação anterior
    }
    if (::symlink(nome_pty_.c_str(), link.c_str()) != 0) {
        gzwarn << "robo_simulado: não criei o link " << link << ", use " << nome_pty_ << "\n";
        link_serial_ = nome_pty_;
    }
    return true;
}

void RoboSimulado::PreUpdate(const UpdateInfo& info, EntityComponentManager& ecm) {
    if (!pronto_) return;
    const double t = segundos(info.simTime);
    if (t < t_anterior_) {                 // botão de reset: o tempo voltou
        t_ultimo_quadro_ = -1e9;
        t_proximo_frame_ = t;
        fila_.clear();
    }
    t_anterior_ = t;

    ler_serial(t);   // lê mesmo pausado, senão a pty enche e a ponte acusa falha de escrita
    if (info.paused) return;

    // Watchdog do firmware: sem quadro válido, tudo em inércia
    const bool watchdog = t - t_ultimo_quadro_ > watchdog_s_;
    if (watchdog != em_watchdog_) {
        em_watchdog_ = watchdog;
        if (watchdog) {
            std::printf("robo_simulado: watchdog, %.0f ms sem quadro válido: rodas em inércia\n",
                        watchdog_s_ * 1000.0);
        } else {
            std::printf("robo_simulado: quadros chegando, motores ligados\n");
        }
        std::fflush(stdout);
    }

    for (int i = 0; i < NUM_RODAS; ++i) {
        const auto w = juntas_[i].Velocity(ecm);
        const double w_roda = (w && !w->empty()) ? w->front() : 0.0;
        juntas_[i].SetForce(ecm, {watchdog ? 0.0 : torque(i, w_roda)});
    }
}

void RoboSimulado::ler_serial(double t) {
    char buf[512];
    ssize_t n;
    while ((n = ::read(fd_mestre_, buf, sizeof(buf))) > 0) resto_.append(buf, (size_t)n);

    size_t fim;
    while ((fim = resto_.find('\n')) != std::string::npos) {
        const std::string linha = resto_.substr(0, fim);
        resto_.erase(0, fim + 1);
        if (linha.empty()) continue;
        // Como no firmware, linha maior que o buffer de 64 bytes é descartada
        if (linha.size() < 64 && processar(linha, t)) continue;
        if (++quadros_ruins_ % 50 == 1) {  // avisa no 1o e depois a cada 50, sem inundar o terminal
            std::printf("robo_simulado: quadro inválido (%d até agora): %s\n",
                        quadros_ruins_, linha.c_str());
            std::fflush(stdout);
        }
    }
    if (resto_.size() >= 64) resto_.clear();
}

/** Mesmas regras do processar() do firmware. Devolve false se o quadro veio corrompido. */
bool RoboSimulado::processar(const std::string& linha, double t) {
    const size_t inicio = linha.find('#'), estrela = linha.rfind('*');
    if (inicio == std::string::npos || estrela == std::string::npos || estrela < inicio) return false;

    const std::string corpo = linha.substr(inicio + 1, estrela - inicio - 1);
    uint8_t chk = 0;
    for (char c : corpo) chk ^= (uint8_t)c;
    if (chk != (uint8_t)std::strtol(linha.c_str() + estrela + 1, nullptr, 16)) return false;

    if (corpo[0] == 'P') {                 // calibração: PWM direto, de -255 a 255
        int pwm[NUM_RODAS];
        if (std::sscanf(corpo.c_str(), "P,%d,%d,%d", &pwm[0], &pwm[1], &pwm[2]) != 3) return false;
        for (int i = 0; i < NUM_RODAS; ++i) {
            // Inverso do mapeamento do firmware: até o PWM_MINIMO a roda não vence o atrito
            const int p = std::min(std::abs(pwm[i]), 255);
            const double f = p > pwm_minimo_ ? (double)(p - pwm_minimo_) / (255 - pwm_minimo_) : 0.0;
            definir(i, std::copysign(f * v_max_m_s_ * 1000.0, (double)pwm[i]), 'A');
        }
    } else if (corpo[0] == 'V') {
        int v[NUM_RODAS];
        char m[NUM_RODAS];
        if (std::sscanf(corpo.c_str(), "V,%d,%d,%d,%c,%c,%c", &v[0], &v[1], &v[2],
                        &m[0], &m[1], &m[2]) != 6) return false;
        for (int i = 0; i < NUM_RODAS; ++i) definir(i, v[i], m[i]);
    } else {
        return false;
    }
    t_ultimo_quadro_ = t;
    return true;
}

/** Como o aplicar() do firmware: I e F valem como estão, qualquer outra letra é ativo. */
void RoboSimulado::definir(int roda, double v_mm_s, char modo) {
    if (modo != 'I' && modo != 'F') modo = 'A';
    // Velocidade zero vira inércia; quem quer freio manda o modo F
    if (modo == 'A' && std::abs(v_mm_s) <= 1.0) modo = 'I';
    modo_[roda] = modo;
    // Acima de V_MAX_MM_S o PWM satura em 100%
    w_ref_[roda] = std::clamp(v_mm_s / 1000.0, -v_max_m_s_, v_max_m_s_) / raio_roda_m_;
}

double RoboSimulado::torque(int roda, double w) const {
    double tau;
    switch (modo_[roda]) {
        case 'A': tau = ganho_Nms_ * (w_ref_[roda] - w); break;
        case 'F': tau = -ganho_Nms_ * w; break;
        default:  return 0.0;              // inércia: roda livre
    }
    return std::clamp(tau, -torque_max_Nm_, torque_max_Nm_);
}

void RoboSimulado::PostUpdate(const UpdateInfo& info, const EntityComponentManager& ecm) {
    if (!pronto_ || info.paused) return;
    const double t = segundos(info.simTime);

    if (t >= t_proximo_frame_) {
        capturar(t, ecm);
        t_proximo_frame_ += periodo_s_;
        if (t_proximo_frame_ < t) t_proximo_frame_ = t + periodo_s_;
    }
    // O frame leva o carimbo da captura, mas só chega na ponte latencia_s depois
    while (!fila_.empty() && fila_.front().t_entrega <= t) {
        const std::string& texto = fila_.front().texto;
        ::sendto(fd_udp_, texto.data(), texto.size(), 0, (const sockaddr*)&destino_, sizeof(destino_));
        fila_.pop_front();
    }
}

/** Fotografa o robô e o alvo agora; o frame entra na fila e sai depois da latência. */
void RoboSimulado::capturar(double t, const EntityComponentManager& ecm) {
    if (alvo_ == kNullEntity || !ecm.HasEntity(alvo_)) {
        alvo_ = ecm.EntityByComponents(components::Name(nome_alvo_), components::Model());
    }
    if (alvo_ == kNullEntity) {            // sem alvo não há quadro completo para mandar
        if (!sem_alvo_avisado_) {
            std::printf("robo_simulado: não achei o modelo [%s], a visão fica muda\n",
                        nome_alvo_.c_str());
            std::fflush(stdout);
            sem_alvo_avisado_ = true;
        }
        return;
    }
    if (uniforme_(rng_) < perda_frames_) return;   // frame perdido: a visão não manda nada

    const gz::math::Pose3d robo = worldPose(modelo_.Entity(), ecm);
    const gz::math::Pose3d alvo = worldPose(alvo_, ecm);
    double x = robo.Pos().X() + ruido_pos_m_ * normal_(rng_);
    double y = robo.Pos().Y() + ruido_pos_m_ * normal_(rng_);
    double theta = robo.Rot().Yaw() + ruido_ang_rad_ * normal_(rng_);
    theta = std::atan2(std::sin(theta), std::cos(theta));
    if (uniforme_(rng_) < deteccao_errada_) {      // deteção errada: o robô "aparece" longe
        const double a = 2.0 * M_PI * uniforme_(rng_), d = 0.5 + 0.5 * uniforme_(rng_);
        x += d * std::cos(a);
        y += d * std::sin(a);
    }
    const double ax = alvo.Pos().X() + ruido_pos_m_ * normal_(rng_);
    const double ay = alvo.Pos().Y() + ruido_pos_m_ * normal_(rng_);

    // Sem arredondar: a ponte trata pose idêntica em frames seguidos como visão travada
    const double k = milimetros_ ? 1000.0 : 1.0;
    char texto[160];
    std::snprintf(texto, sizeof(texto), "pose %.5f %.5f %.5f %.5f %.5f %.4f",
                  x * k, y * k, theta, ax * k, ay * k, t);
    fila_.push_back({t + latencia_s_, texto});
}

}  // namespace robo_teste

GZ_ADD_PLUGIN(robo_teste::RoboSimulado, gz::sim::System,
              robo_teste::RoboSimulado::ISystemConfigure,
              robo_teste::RoboSimulado::ISystemPreUpdate,
              robo_teste::RoboSimulado::ISystemPostUpdate)
