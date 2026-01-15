#include "Encoder.h"
#include <iostream>

Encoder::Encoder(int pinA, int pinB, int cpr)
    : a(pinA), b(pinB), cpr(cpr) {}

void Encoder::begin()
{
    gpioSetMode(a, PI_INPUT);
    gpioSetMode(b, PI_INPUT);
    gpioSetPullUpDown(a, PI_PUD_UP);
    gpioSetPullUpDown(b, PI_PUD_UP);

    position = 0;
    lastPos = 0;
    lastTime = gpioTick();
    lastSpeed = 0.0f;

    gpioSetAlertFuncEx(a, Encoder::callback, this);
}

void Encoder::callback(int gpio, int level, uint32_t tick, void* user)
{
    if (level == 2) return;

    Encoder* enc = static_cast<Encoder*>(user);

    if (gpioRead(enc->b) != level)
        enc->position++;
    else
        enc->position--;
}

long Encoder::getPosition()
{
    return position;
}

void Encoder::reset()
{
    position = 0;
    lastPos = 0;
    lastTime = gpioTick();
    lastSpeed = 0.0f;
}

float Encoder::getSpeed()
{
    uint32_t now = gpioTick();
    long currentPos = position;
    
    // Calcular tempo decorrido desde última medição
    uint32_t dt_us = now - lastTime;
    float dt = dt_us / 1e6f;

    // Proteção contra dt zero ou muito pequeno
    if (dt < 0.001f) {
        return lastSpeed;
    }

    // SEMPRE calcular velocidade baseado na posição atual
    long deltaPos = currentPos - lastPos;
    float speed = deltaPos / dt;
    
    // Atualizar valores para próxima medição
    lastPos = currentPos;
    lastTime = now;
    lastSpeed = speed;

    return speed;
}