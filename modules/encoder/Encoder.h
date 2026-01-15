#pragma once
#include <pigpio.h>
#include <cstdint>

class Encoder {
public:
    Encoder(int pinA, int pinB, int cpr);

    void begin();

    long getPosition();
    void reset();
    float getSpeed(); // pulsos por segundo

private:
    int a, b;
    int cpr;

    volatile long position = 0;
    long lastPos = 0;
    uint32_t lastTime = 0;
    long lastSpeed = 0;

    static void callback(int gpio, int level, uint32_t tick, void* user);
};
