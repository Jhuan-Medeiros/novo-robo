#include "cameraReceive.h"
#include <atomic>
#include <thread>
#include <chrono>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <cstdio>
#include <iostream>

// Variáveis atômicas para compartilhar entre threads
static std::atomic<char> g_comando{'N'};
static std::atomic<int> g_erro_x{0};
static std::atomic<int> g_distancia{0};
static std::atomic<bool> g_alvo_detectado{false};
static std::atomic<uint32_t> g_ultima_atualizacao{0};

static void loopRecepcaoCamera()
{
    // Cria socket UDP na porta 5006
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0)
    {
        std::cerr << "✗ Erro ao criar socket da câmera!" << std::endl;
        return;
    }

    // Configura timeout para não travar
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000; // 100ms
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(5006);
    addr.sin_addr.s_addr = htonl(INADDR_ANY); // Aceita de qualquer IP (incluindo localhost)

    if (bind(sock, (sockaddr *)&addr, sizeof(addr)) < 0)
    {
        std::cerr << "✗ Erro ao fazer bind na porta 5006!" << std::endl;
        std::cerr << "  Possível causa: porta já em uso" << std::endl;
        close(sock);
        return;
    }

    std::cout << "✓ Recepção de câmera iniciada (localhost:5006)" << std::endl;

    char buf[256];
    sockaddr_in src{};
    socklen_t srclen = sizeof(src);
    uint32_t pacotes_recebidos = 0;

    while (true)
    {
        ssize_t n = recvfrom(sock, buf, sizeof(buf) - 1, 0,
                             (sockaddr *)&src, &srclen);
        if (n > 0)
        {
            buf[n] = '\0';

            // Parse: "COMANDO,ERRO_X,DISTANCIA"
            // Exemplo: "D,145,50" ou "P,2,30" ou "N,0,0"
            char cmd;
            int erro, dist;

            if (sscanf(buf, "%c,%d,%d", &cmd, &erro, &dist) == 3)
            {
                g_comando.store(cmd);
                g_erro_x.store(erro);
                g_distancia.store(dist);
                g_alvo_detectado.store(true); // câmera está viva, mesmo sem alvo
                g_ultima_atualizacao.store(time(nullptr));

                pacotes_recebidos++;

                // Debug mais detalhado
                if (pacotes_recebidos % 100 == 0)
                {
                    std::cout << "📷 Camera: " << pacotes_recebidos
                              << " pacotes | Cmd=" << cmd
                              << " | Erro=" << erro
                              << " | Alvo=" << (cmd != 'N' ? "SIM" : "NÃO") << std::endl;
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    close(sock);
}

void iniciarRecepcaoCamera()
{
    std::thread(loopRecepcaoCamera).detach();

    // Aguarda um pouco para garantir que a thread iniciou
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

CameraState lerCamera()
{
    return CameraState{
        g_comando.load(),
        g_erro_x.load(),
        g_distancia.load(),
        g_alvo_detectado.load()};
}