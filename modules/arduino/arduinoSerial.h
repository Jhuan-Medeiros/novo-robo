#pragma once

// Estrutura com os dados dos sensores
struct DadosSensores {
    float ultraEsq;      // Distância ultrassom esquerdo (cm)
    float ultraDir;      // Distância ultrassom direito (cm)
    int laserFrente;     // Distância laser frontal (cm)
    int laserTras;       // Distância laser traseiro (cm)
    int estadoLinha;     // 0=sem linha, 1=dir, 2=esq, 3=ambos
    bool valido;         // true se já recebeu dados válidos
};

// Funções de interface
int configurarSerial(const char* porta);
void enviarServos(int fd, int pinca, int angulo);
std::string lerSensores(int fd);

// NOVA: Acesso direto aos dados (ZERO DELAY, thread-safe)
DadosSensores obterDadosSensores();