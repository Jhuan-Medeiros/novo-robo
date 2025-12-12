#include <iostream>
#include <pigpio.h>
#include <stdlib.h>
#include "../PC_controle/receive.h" 
#define _USE_MATH_DEFINES 
#include <cmath>

#define M1_IN1 27
#define M1_IN2 22
#define M1_PWM 17

#define M2_IN1 10
#define M2_IN2 9
#define M2_PWM 11

#define M3_IN1 5
#define M3_IN2 6
#define M3_PWM 13

const float center = 0.133f;
const float radius = 0.033f;

const float sMax = 0.30f;
const float rMax = 1.0f;

const float CPR = 472.0f;

struct spdWheels {
    float w1;
    float w2;
    float w3;
};

struct Encoder
{
    int C1;
    int C2;
    long pos;
};

struct motor {
    int in1;
    int in2;
    int pwmPin;
    int value;
};

motor motors[3] = {
    { M1_IN1, M1_IN2, M1_PWM, 0},
    { M2_IN1, M2_IN2, M2_PWM, 0},
    { M3_IN1, M3_IN2, M3_PWM, 0}
};

void setupMotors(const motor& m) {
    gpioSetMode(m.in1, PI_OUTPUT);
    gpioSetMode(m.in2, PI_OUTPUT);
    gpioSetMode(m.pwmPin, PI_OUTPUT);
    gpioSetPWMfrequency(m.pwmPin, 5000);
    gpioSetPWMrange(m.pwmPin, 255);
}

float deadzone(float value, float zone = 0.20f) {
    if (std::abs(value) < zone) return 0.0f;
    return value;
}

spdWheels conversion(float lx, float ly, float rx){
    float vx = ly * sMax;
    float vy = lx * sMax;
    float w = rx * rMax;

    vx = deadzone(vx, 0.10f);
    vy = deadzone(vy, 0.10f);
    w = deadzone(w, 0.10f);

    spdWheels spd;

    const float L = center;
    const float r = radius;

    const float sqrt3_over2 = std::sqrt(3.0f) / 2.0f;

    spd.w1 = (-sqrt3_over2 / r) * vx + (0.5f / r) * vy + (L / r) * w;
    spd.w2 = (0.0f) * vx + (-1.0f / r) * vy + (L / r) * w;
    spd.w3 = (sqrt3_over2 / r) * vx + (0.5f / r) * vy + (L / r) * w;

    return spd;
}

void setMotor(int IN1, int IN2, int pwmPin, float speed) {

    int pwmValue = int(std::abs(speed) * 150.0f);

    if(pwmValue > 255) pwmValue = 255;

    if (speed > 0.1f)
    {
        gpioWrite(IN1, PI_HIGH);
        gpioWrite(IN2, PI_LOW);
    } else if (speed < -0.1f) {
        gpioWrite(IN1, PI_LOW);
        gpioWrite(IN2, PI_HIGH);
    } else {
        gpioWrite(IN1, PI_LOW);
        gpioWrite(IN2, PI_LOW);
        speed = 0;
    }
    gpioPWM(pwmPin, pwmValue);
}

int main()
{
    if (gpioInitialise() < 0) return 1;

    for (int i = 0; i < 3; i++)
    {
        setupMotors(motors[i]);
    }
    

    iniciarRecepcaoControle(); 
    while (true) {
        controleState c = lerControle(); 

        float lx = c.lx;
        float ly = c.ly;
        float rx = c.rx;

        spdWheels s = conversion(lx, ly, rx);

        setMotor(M1_IN1, M1_IN2, M1_PWM, s.w1);
        setMotor(M2_IN1, M2_IN2, M2_PWM, s.w2);
        setMotor(M3_IN1, M3_IN2, M3_PWM, s.w3);

        gpioDelay(20000); // 20ms
    }

    gpioTerminate();
    return 0;
}
