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

static std::atomic<int> g_dUp{0};
static std::atomic<int> g_dDown{0};
static std::atomic<int> g_dLeft{0};
static std::atomic<int> g_dRight{0};

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
            int up, down, left, right;
            if (sscanf(buf, "%f,%f,%f,%f,%d,%d,%d,%d", &lx, &ly, &rx, &ry, &up, &down, &left, &right) == 8) {
                g_lx = lx;
                g_ly = ly;
                g_rx = rx;
                g_ry = ry;
                
                g_dUp = up;
                g_dDown = down;
                g_dLeft = left;
                g_dRight = right;

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
        g_ry.load(),

        g_dUp.load(),
        g_dDown.load(),
        g_dLeft.load(),
        g_dRight.load()
    };
}
