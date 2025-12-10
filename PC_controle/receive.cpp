// receive.cpp
#include "receive.h"
#include <atomic>
#include <thread>
#include <chrono>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <cstdio>

static std::atomic<float> g_lx{0.0f};
static std::atomic<float> g_ly{0.0f};
static std::atomic<float> g_rx{0.0f};
static std::atomic<float> g_ry{0.0f};

static void loopRecepcao()
{
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(5005);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    bind(sock, (sockaddr*)&addr, sizeof(addr));

    char buf[1024];
    sockaddr_in src{};
    socklen_t srclen = sizeof(src);

    while (true) {
        ssize_t n = recvfrom(sock, buf, sizeof(buf)-1, 0,
                             (sockaddr*)&src, &srclen);
        if (n > 0) {
            buf[n] = '\0';
            float lx, ly, rx, ry;
            if (sscanf(buf, "%f,%f,%f,%f", &lx, &ly, &rx, &ry) == 4) {
                g_lx = lx;
                g_ly = ly;
                g_rx = rx;
                g_ry = ry;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void iniciarRecepcaoControle()
{
    std::thread(loopRecepcao).detach();
}

controleState lerControle()
{
    return controleState{
        g_lx.load(),
        g_ly.load(),
        g_rx.load(),
        g_ry.load()
    };
}
