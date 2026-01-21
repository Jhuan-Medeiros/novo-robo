#include "Encoder.h"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <chrono>

// Inicialização das variáveis estáticas
int Encoder::serialFd = -1;
std::atomic<bool> Encoder::threadAtiva{false};
std::atomic<int32_t> Encoder::positionsShared[4] = {0, 0, 0, 0};

Encoder::Encoder(int index)
    : encoderIndex(index),
      position(0),
      lastTime(0),
      lastPos(0),
      lastSpeed(0.0f)
{
    if (index < 0 || index > 3) {
        std::cerr << "ERRO: Encoder index inválido: " << index << std::endl;
        encoderIndex = 0;
    }
}

void Encoder::begin()
{
    lastTime = getMicros();
    position = 0;
    lastPos = 0;
    lastSpeed = 0.0f;
}

bool Encoder::iniciarComunicacaoSerial(const char* porta)
{
    // Abre a porta serial UART (normalmente /dev/ttyS0 ou /dev/serial0 na Raspberry)
    serialFd = open(porta, O_RDWR | O_NOCTTY | O_NDELAY);
    if (serialFd == -1) {
        std::cerr << "ERRO: Não foi possível abrir " << porta << std::endl;
        std::cerr << "Verifique se a UART está habilitada (raspi-config)" << std::endl;
        return false;
    }

    // Configura a porta serial UART
    struct termios options;
    tcgetattr(serialFd, &options);
    
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    
    // Configurações 8N1 (8 bits, sem paridade, 1 stop bit)
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;   // Sem paridade
    options.c_cflag &= ~CSTOPB;   // 1 stop bit
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;       // 8 bits
    options.c_cflag &= ~CRTSCTS;  // Desabilita hardware flow control
    
    // Modo RAW
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL);
    options.c_oflag &= ~OPOST;
    
    // Timeout e bloqueio
    options.c_cc[VMIN] = 1;   // Lê no mínimo 1 byte
    options.c_cc[VTIME] = 0;  // Sem timeout
    
    tcsetattr(serialFd, TCSANOW, &options);
    tcflush(serialFd, TCIOFLUSH);  // Limpa buffers
    
    std::cout << "UART configurada em " << porta << " @ 115200 baud" << std::endl;
    std::cout << "Aguardando dados dos encoders..." << std::endl;
    
    // Aguarda um pouco para estabilizar
    usleep(500000);  // 500ms
    
    // Inicia a thread de leitura
    threadAtiva = true;
    std::thread(threadLeituraSerial).detach();
    
    std::cout << "Thread de leitura UART iniciada!" << std::endl;
    
    return true;
}

void Encoder::pararComunicacaoSerial()
{
    threadAtiva = false;
    usleep(100000);  // Aguarda thread terminar
    
    if (serialFd != -1) {
        close(serialFd);
        serialFd = -1;
    }
}

void Encoder::threadLeituraSerial()
{
    uint8_t header;
    int32_t positions[4];
    
    while (threadAtiva) {
        // Busca o header de sincronização (0xAA)
        if (read(serialFd, &header, 1) != 1) {
            continue;
        }
        
        if (header != 0xAA) {
            continue;  // Não é o header, continua procurando
        }
        
        // Lê os 4 encoders (4 bytes cada = 16 bytes total)
        int bytesRead = read(serialFd, positions, sizeof(positions));
        
        if (bytesRead == sizeof(positions)) {
            // Atualiza as posições compartilhadas (atomic)
            for (int i = 0; i < 4; i++) {
                positionsShared[i].store(positions[i]);
            }
        } else {
            // Erro na leitura, descarta e tenta sincronizar novamente
            tcflush(serialFd, TCIFLUSH);
        }
    }
}

long Encoder::getPosition()
{
    // Lê a posição do array compartilhado
    return positionsShared[encoderIndex].load();
}

float Encoder::getSpeed()
{
    uint32_t now = getMicros();
    uint32_t dt_us = now - lastTime;

    // Evita divisão por zero e cálculo muito frequente
    if (dt_us < 1000) {
        return lastSpeed;
    }

    float dt = dt_us / 1e6f;  // Converte para segundos

    long pos = getPosition();
    long delta = pos - lastPos;

    float speed = delta / dt;  // pulsos/segundo

    lastPos = pos;
    lastTime = now;
    lastSpeed = speed;

    return speed;
}

void Encoder::reset()
{
    // Reset do offset local
    position = 0;
    lastPos = positionsShared[encoderIndex].load();  // Sincroniza com posição atual
    lastTime = getMicros();
    lastSpeed = 0.0f;
}

uint32_t Encoder::getMicros()
{
    static auto start = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::microseconds>(now - start).count();
}