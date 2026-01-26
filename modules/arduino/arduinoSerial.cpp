#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <string>
#include <cstring>
#include <thread>
#include <atomic>
#include <mutex>
#include <algorithm>

struct DadosSensores {
    float ultraEsq = 0.0f;
    float ultraDir = 0.0f;
    int laserFrente = 0;
    int laserTras = 0;
    int estadoLinha = 0;
    bool valido = false;
};

static std::atomic<bool> threadAtiva{false};
static std::mutex dadosMutex;
static DadosSensores dadosAtuais;
static int serialFd = -1;

static void threadLeituraSerial() {
    std::string buffer;
    char temp[256];
    
    while (threadAtiva) {
        int n = read(serialFd, temp, sizeof(temp) - 1);
        
        if (n > 0) {
            temp[n] = '\0';
            buffer += std::string(temp);
            
            size_t pos;
            while ((pos = buffer.find('\n')) != std::string::npos) {
                std::string linha = buffer.substr(0, pos);
                buffer.erase(0, pos + 1);
                
                linha.erase(std::remove_if(linha.begin(), linha.end(), 
                    [](char c) { return c == '\r' || c == ' ' || c == '\t'; }), linha.end());
                
                float ue, ud;
                int lf, lt, el;
                
                if (sscanf(linha.c_str(), "%f,%f,%d,%d,%d", &ue, &ud, &lf, &lt, &el) == 5) {
                    std::lock_guard<std::mutex> lock(dadosMutex);
                    dadosAtuais.ultraEsq = ue;
                    dadosAtuais.ultraDir = ud;
                    dadosAtuais.laserFrente = lf;
                    dadosAtuais.laserTras = lt;
                    dadosAtuais.estadoLinha = el;
                    dadosAtuais.valido = true;
                }
            }
            
            if (buffer.length() > 512) {
                buffer.clear();
            }
        }
        
        usleep(1000);
    }
}

int configurarSerial(const char* porta) {
    serialFd = open(porta, O_RDWR | O_NOCTTY);
    if (serialFd == -1) return -1;

    struct termios options;
    tcgetattr(serialFd, &options);
    
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL);
    options.c_oflag &= ~OPOST;
    
    // Timeout mínimo
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 1;
    
    tcsetattr(serialFd, TCSANOW, &options);
    
    // Aguarda reset do Arduino
    std::cout << "Aguardando Arduino resetar..." << std::endl;
    usleep(2500000);
    tcflush(serialFd, TCIOFLUSH);
    
    // Inicia thread de leitura
    threadAtiva = true;
    std::thread(threadLeituraSerial).detach();
    
    std::cout << "Arduino pronto! Thread de leitura iniciada." << std::endl;
    
    return serialFd;
}

void enviarServos(int fd, int pinca, int angulo) {
    std::string comando = "P" + std::to_string(pinca) + "A" + std::to_string(angulo) + "\n";
    write(fd, comando.c_str(), comando.length());
    // Removido tcdrain() para evitar delay
}

// Função de leitura instantânea (SEM DELAY)
std::string lerSensores(int fd) {
    std::lock_guard<std::mutex> lock(dadosMutex);
    
    if (!dadosAtuais.valido) {
        return "Aguardando dados...";
    }
    
    char buffer[128];
    snprintf(buffer, sizeof(buffer), "%.1f,%.1f,%d,%d,%d",
             dadosAtuais.ultraEsq,
             dadosAtuais.ultraDir,
             dadosAtuais.laserFrente,
             dadosAtuais.laserTras,
             dadosAtuais.estadoLinha);
    
    return std::string(buffer);
}

// Nova função para acesso direto aos dados (ZERO DELAY)
DadosSensores obterDadosSensores() {
    std::lock_guard<std::mutex> lock(dadosMutex);
    return dadosAtuais;
}