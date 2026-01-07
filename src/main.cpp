#include <iostream>
#include <pigpio.h>
#include <stdlib.h>
#include "../PC_controle/receive.h"
#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>

// ================= PINOS =================

#define M1_IN1 27
#define M1_IN2 22
#define M1_PWM 17
#define M1_C1 23
#define M1_C2 24

#define M2_IN1 11
#define M2_IN2 9
#define M2_PWM 10
#define M2_C1 25
#define M2_C2 8

#define M3_IN1 13
#define M3_IN2 6
#define M3_PWM 5
#define M3_C1 7
#define M3_C2 1

// ================= PARAMETROS =================

const float center = 0.103f;
const float radius = 0.033f;

const float sMax = 0.30f;
const float rMax = 1.0f;

const float CPR = 472.0f;

// ================= STRUCTS =================

struct spdWheels
{
    float w1;
    float w2;
    float w3;
};

struct motor
{
    int in1;
    int in2;
    int pwmPin;
    int c1;
    int c2;

    volatile int32_t pos = 0;
    volatile uint8_t lastAB = 0;

    int32_t prevPos = 0;
    uint32_t prevTime = 0;

    float prevError = 0;
    float integralError = 0;

    float kp = 0.6f;
    float ki = 0.5f;
    float kd = 0.05f;
};

// ================= QUADRATURA =================

const int8_t quadTable[16] = {
    0, -1, +1, 0,
    +1, 0, 0, -1,
    -1, 0, 0, +1,
    0, +1, -1, 0};

// ================= MOTORES =================

motor motors[3] = {
    {M1_IN1, M1_IN2, M1_PWM, M1_C1, M1_C2},
    {M2_IN1, M2_IN2, M2_PWM, M2_C1, M2_C2},
    {M3_IN1, M3_IN2, M3_PWM, M3_C1, M3_C2}};

// ================= SETUP MOTOR =================

void setupMotors(const motor &m)
{
    gpioSetMode(m.in1, PI_OUTPUT);
    gpioSetMode(m.in2, PI_OUTPUT);
    gpioSetMode(m.pwmPin, PI_OUTPUT);

    gpioSetMode(m.c1, PI_INPUT);
    gpioSetMode(m.c2, PI_INPUT);

    gpioSetPullUpDown(m.c1, PI_PUD_UP);
    gpioSetPullUpDown(m.c2, PI_PUD_UP);

    gpioSetPWMfrequency(m.pwmPin, 5000);
    gpioSetPWMrange(m.pwmPin, 255);
}

// ================= ENCODER ISR =================

void encoderISR(int gpio, int level, uint32_t tick, void *user)
{
    if (level != 0 && level != 1)
        return;

    motor *m = (motor *)user;

    uint32_t levels = gpioRead_Bits_0_31();
    uint8_t A = (levels >> m->c1) & 1;
    uint8_t B = (levels >> m->c2) & 1;

    uint8_t currentAB = (A << 1) | B;
    uint8_t index = (m->lastAB << 2) | currentAB;

    m->pos += quadTable[index];
    m->lastAB = currentAB;
}

// ================= PID =================

float computePID(motor *m, float targetSpeed)
{
    uint32_t now = gpioTick();
    float dt = (now - m->prevTime) * 1e-6f;

    if (dt < 0.001f)
        return 0;

    int32_t delta = m->pos - m->prevPos;
    float currentSpeed = delta / dt;

    float error = targetSpeed - currentSpeed;

    m->integralError += error * dt;
    m->integralError = std::clamp(m->integralError, -200.0f, 200.0f);

    float derivative = (error - m->prevError) / dt;

    float output =
        m->kp * error +
        m->ki * m->integralError +
        m->kd * derivative;

    output = std::clamp(output, -255.0f, 255.0f);

    m->prevPos = m->pos;
    m->prevError = error;
    m->prevTime = now;

    if (fabs(targetSpeed) < 5.0f)
    {
        m->integralError = 0;
        return 0;
    }

    return output;
}

// ================= AUX =================

float deadzone(float value, float zone = 0.20f)
{
    if (fabs(value) < zone)
        return 0;
    return value;
}

// ================= CINEMATICA =================

spdWheels conversion(float lx, float ly, float rx)
{
    // 1. Inversão de Eixo (Muitos controles o Y é invertido: para cima é negativo)
    // Se o seu controle já estiver "certo", remova o sinal de menos.
    float vx_raw = lx * sMax;
    float vy_raw = -ly * sMax; // Tente com '-' se o 'pra cima' estiver indo para trás
    float w = rx * rMax;

    // 2. COMPENSAÇÃO DE ROTAÇÃO (O segredo está aqui)
    // Ajuste este valor (em radianos) até que 'pra cima' no stick seja 'frente' no robô.
    // Como a diagonal (45°) está sendo a frente, tente -45° ou +45° (M_PI / 4.0f)
    float headingOffset = -M_PI / 4.0f;

    float vx = vx_raw * cos(headingOffset) - vy_raw * sin(headingOffset);
    float vy = vx_raw * sin(headingOffset) + vy_raw * cos(headingOffset);

    // 3. Deadzone
    vx = deadzone(vx, 0.10f);
    vy = deadzone(vy, 0.10f);
    w = deadzone(w, 0.10f);

    const float r = radius;
    const float L = center;

    // 4. Ângulos das rodas (Mantendo o padrão 30°, 150°, 270°)
    const float th1 = M_PI / 6.0f;
    const float th2 = 5.0f * M_PI / 6.0f;
    const float th3 = 3.0f * M_PI / 2.0f;

    return {
        (-sin(th1) * vx + cos(th1) * vy + L * w) / r,
        (-sin(th2) * vx + cos(th2) * vy + L * w) / r,
        (-sin(th3) * vx + cos(th3) * vy + L * w) / r};
}

// ================= MOTOR =================

void setMotor(int IN1, int IN2, int pwmPin, float val)
{
    val = std::clamp(val, -255.0f, 255.0f);
    int pwm = abs((int)val);

    if (val > 0.1f)
    {
        gpioWrite(IN1, 1);
        gpioWrite(IN2, 0);
    }
    else if (val < -0.1f)
    {
        gpioWrite(IN1, 0);
        gpioWrite(IN2, 1);
    }
    else
    {
        gpioWrite(IN1, 0);
        gpioWrite(IN2, 0);
        pwm = 0;
    }

    gpioPWM(pwmPin, pwm);
}

// ================= MAIN =================

int main()
{
    if (gpioInitialise() < 0)
        return 1;

    for (int i = 0; i < 3; i++)
    {
        setupMotors(motors[i]);

        uint8_t A = gpioRead(motors[i].c1);
        uint8_t B = gpioRead(motors[i].c2);
        motors[i].lastAB = (A << 1) | B;

        gpioSetAlertFuncEx(motors[i].c1, encoderISR, &motors[i]);
        gpioSetAlertFuncEx(motors[i].c2, encoderISR, &motors[i]);
    }

    iniciarRecepcaoControle();

    const float RAD_TO_PULSE = CPR / (2.0f * M_PI);

    while (true)
    {
        controleState c = lerControle();

        spdWheels s = conversion(c.lx, c.ly, c.rx);

        setMotor(M1_IN1, M1_IN2, M1_PWM,
                 computePID(&motors[0], s.w1 * RAD_TO_PULSE));

        setMotor(M2_IN1, M2_IN2, M2_PWM,
                 computePID(&motors[1], s.w2 * RAD_TO_PULSE));

        setMotor(M3_IN1, M3_IN2, M3_PWM,
                 computePID(&motors[2], s.w3 * RAD_TO_PULSE));

        gpioDelay(20000);
    }

    gpioTerminate();
    return 0;
}
