#include <iostream>
#include <pigpio.h>
#include <stdlib.h>
#include "../PC_controle/receive.h"

#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>

// ===== BIBLIOTECAS =====
#include "../modules/encoder/Encoder.h"
#include "../libs/PID/PID_v1.h"

// ================= PINOS =================

// Motores 1, 2, 3 (omnidirecional com PID)
#define M1_IN1 27
#define M1_IN2 17
#define M1_PWM 22
#define M1_C1 23
#define M1_C2 24

#define M2_IN1 11
#define M2_IN2 9
#define M2_PWM 10
#define M2_C1 8
#define M2_C2 25

#define M3_IN1 5
#define M3_IN2 6
#define M3_PWM 13
#define M3_C1 12
#define M3_C2 7

// Motor 4 (subida - sem PID)
#define M4_IN1 21
#define M4_IN2 20
#define M4_PWM 16
#define M4_C1 19
#define M4_C2 26

// ================= PARAMETROS =================

const float center = 0.103f;
const float radius = 0.033f;

const float sMax = 0.06f; // Velocidade bem reduzida
const float rMax = 0.10f; // Rotação bem lenta

// Encoder AB (x4)
const float CPR_PHYSICAL = 472.0f;
const float CPR_EFFECTIVE = CPR_PHYSICAL * 4.0f;

// ================= ESTRUTURAS =================

struct spdWheels
{
    float w1;
    float w2;
    float w3;
};

// ================= ENCODERS =================

Encoder enc1(M1_C1, M1_C2, CPR_EFFECTIVE);
Encoder enc2(M2_C1, M2_C2, CPR_EFFECTIVE);
Encoder enc3(M3_C1, M3_C2, CPR_EFFECTIVE);
Encoder enc4(M4_C1, M4_C2, CPR_EFFECTIVE); // Encoder do motor de subida

// ================= PID VARS =================

double in1 = 0, out1 = 0, sp1 = 0;
double in2 = 0, out2 = 0, sp2 = 0;
double in3 = 0, out3 = 0, sp3 = 0;

// ================= PID CONTROLLERS =================
// Ganhos reduzidos para evitar oscilação

// Motor 1 - suavizado
PID pid1(&in1, &out1, &sp1, 0.5, 0.01, 0.0, DIRECT);

// Motor 2 - suavizado mas um pouco mais forte
PID pid2(&in2, &out2, &sp2, 0.5, 0.01, 0.0, DIRECT);

// Motor 3 - bem suavizado (era o mais forte)
PID pid3(&in3, &out3, &sp3, 0.5, 0.01, 0.0, DIRECT);

// ================= FUNÇÕES AUX =================

float deadzone(float value, float zone = 0.10f)
{
    if (std::abs(value) < zone)
        return 0.0f;
    return value;
}

spdWheels conversion(float lx, float ly, float rx)
{
    // --- PADRONIZAÇÃO DE EIXOS ---
    // lx (horizontal) -> v_lateral (Eixo X)
    // ly (vertical)   -> v_frontal (Eixo Y)
    float vx = lx * sMax; 
    float vy = ly * sMax; 
    float w  = rx * rMax; 

    vx = deadzone(vx, 0.03f);
    vy = deadzone(vy, 0.03f);
    w  = deadzone(w, 0.03f);

    const float r = radius;
    const float L = center;

    // --- GEOMETRIA ---
    // Coloca a Roda 1 exatamente atrás (270 graus ou 3*PI/2)
    float offset = (3.0f * M_PI) / 2.0f; 

    spdWheels spd;

    // Ângulos das rodas (0, 120 e 240 graus + o deslocamento)
    float a1 = 0.0f + offset;                   // 270° (Atrás)
    float a2 = (2.0f * M_PI / 3.0f) + offset;   // 30°  (Frente-Esquerda)
    float a3 = (4.0f * M_PI / 3.0f) + offset;   // 150° (Frente-Direita)

    // --- CINEMÁTICA INVERSA ---
    // A fórmula correta para robôs omni de 3 rodas:
    // Wi = (-sin(alpha) * Vx + cos(alpha) * Vy + L * w) / r
    
    spd.w1 = (-sin(a1) * vx + cos(a1) * vy + L * w) / r;
    spd.w2 = (-sin(a2) * vx + cos(a2) * vy + L * w) / r;
    spd.w3 = (-sin(a3) * vx + cos(a3) * vy + L * w) / r;

    return spd;
}

void setMotor(int IN1, int IN2, int pwmPin, float controlValue)
{
    const int PWM_MIN = 30; // PWM mínimo aumentado

    if (controlValue > 255)
        controlValue = 255;
    if (controlValue < -255)
        controlValue = -255;

    int pwm = abs((int)controlValue);

    // Se PWM muito baixo, zerar completamente para evitar tremor
    if (pwm > 0 && pwm < PWM_MIN)
    {
        pwm = 0; // Mudado: ao invés de forçar PWM_MIN, zera
    }

    if (controlValue > 1.0f)
    {
        gpioWrite(IN1, PI_HIGH);
        gpioWrite(IN2, PI_LOW);
    }
    else if (controlValue < -1.0f)
    {
        gpioWrite(IN1, PI_LOW);
        gpioWrite(IN2, PI_HIGH);
    }
    else
    {
        gpioWrite(IN1, PI_LOW);
        gpioWrite(IN2, PI_LOW);
        pwm = 0;
    }

    gpioPWM(pwmPin, pwm);
}

void setMotorSimple(int IN1, int IN2, int pwmPin, int pwmValue, bool forward)
{
    if (forward)
    {
        gpioWrite(IN1, PI_LOW);
        gpioWrite(IN2, PI_HIGH);
    }
    else
    {
        gpioWrite(IN1, PI_HIGH);
        gpioWrite(IN2, PI_LOW);
    }
    gpioPWM(pwmPin, pwmValue);
}

void stopMotorSimple(int IN1, int IN2, int pwmPin)
{
    gpioWrite(IN1, PI_HIGH);
    gpioWrite(IN2, PI_LOW);
    gpioPWM(pwmPin, 0);
}

// ================= MAIN =================

int main()
{
    if (gpioInitialise() < 0)
        return 1;

    // Motores 1, 2, 3
    gpioSetMode(M1_IN1, PI_OUTPUT);
    gpioSetMode(M1_IN2, PI_OUTPUT);
    gpioSetMode(M1_PWM, PI_OUTPUT);

    gpioSetMode(M2_IN1, PI_OUTPUT);
    gpioSetMode(M2_IN2, PI_OUTPUT);
    gpioSetMode(M2_PWM, PI_OUTPUT);

    gpioSetMode(M3_IN1, PI_OUTPUT);
    gpioSetMode(M3_IN2, PI_OUTPUT);
    gpioSetMode(M3_PWM, PI_OUTPUT);

    // Motor 4 (subida)
    gpioSetMode(M4_IN1, PI_OUTPUT);
    gpioSetMode(M4_IN2, PI_OUTPUT);
    gpioSetMode(M4_PWM, PI_OUTPUT);

    gpioSetPWMfrequency(M1_PWM, 1000);
    gpioSetPWMfrequency(M2_PWM, 1000);
    gpioSetPWMfrequency(M3_PWM, 1000);
    gpioSetPWMfrequency(M4_PWM, 1000);

    gpioSetPWMrange(M1_PWM, 255);
    gpioSetPWMrange(M2_PWM, 255);
    gpioSetPWMrange(M3_PWM, 255);
    gpioSetPWMrange(M4_PWM, 255);

    // Encoders
    enc1.begin();
    enc2.begin();
    enc3.begin();
    enc4.begin();

    gpioDelay(100000); // 100ms para estabilizar

    // PID
    pid1.SetOutputLimits(-255, 255);
    pid2.SetOutputLimits(-255, 255);
    pid3.SetOutputLimits(-255, 255);

    pid1.SetSampleTime(20);
    pid2.SetSampleTime(20);
    pid3.SetSampleTime(20);

    pid1.SetMode(AUTOMATIC);
    pid2.SetMode(AUTOMATIC);
    pid3.SetMode(AUTOMATIC);

    iniciarRecepcaoControle();

    const float RAD_TO_PULSE = CPR_EFFECTIVE / (2.0f * M_PI);

    std::cout << "Sistema iniciado!" << std::endl;
    std::cout << "RAD_TO_PULSE: " << RAD_TO_PULSE << std::endl;

    int loopCount = 0;

    while (true)
    {
        controleState c = lerControle();

        float lx = c.lx;
        float ly = c.ly;
        float rx = c.rx;

        // D-pad tem prioridade (apenas para movimento omni)
        if (c.dUp)
        {
            ly = 1;
            lx = 0;
            rx = 0;
        }
        if (c.dDown)
        {
            ly = -1;
            lx = 0;
            rx = 0;
        }
        if (c.dLeft)
        {
            lx = -1;
            ly = 0;
            rx = 0;
        }
        if (c.dRight)
        {
            lx = 1;
            ly = 0;
            rx = 0;
        }

        // ===== CONTROLE DO MOTOR 4 (SUBIDA) =====
        if (c.l1)
        {
            // L1 pressionado - subir
            setMotorSimple(M4_IN1, M4_IN2, M4_PWM, 255, true);
        }
        else if (c.l2)
        {
            // L2 pressionado - descer
            setMotorSimple(M4_IN1, M4_IN2, M4_PWM, 255, false);
        }
        else
        {
            // Nenhum botão - parar motor 4
            stopMotorSimple(M4_IN1, M4_IN2, M4_PWM);
        }

        // ===== CONTROLE DOS MOTORES 1, 2, 3 (COM PID) =====
        spdWheels s = conversion(lx, ly, rx);

        // Setpoints em pulsos/segundo
        sp1 = s.w1 * RAD_TO_PULSE;
        sp2 = s.w2 * RAD_TO_PULSE;
        sp3 = s.w3 * RAD_TO_PULSE;

        // Leitura dos encoders
        in1 = enc1.getSpeed();
        in2 = enc2.getSpeed();
        in3 = enc3.getSpeed();

        // Se setpoint muito pequeno, zerar tudo e resetar PID
        if (std::abs(sp1) < 10.0f)
        { // Threshold aumentado
            out1 = 0;
            setMotor(M1_IN1, M1_IN2, M1_PWM, 0);
            pid1.SetMode(MANUAL);
            pid1.SetMode(AUTOMATIC); // Reset
        }
        else
        {
            pid1.Compute();
            setMotor(M1_IN1, M1_IN2, M1_PWM, out1);
        }

        if (std::abs(sp2) < 10.0f)
        { // Threshold aumentado
            out2 = 0;
            setMotor(M2_IN1, M2_IN2, M2_PWM, 0);
            pid2.SetMode(MANUAL);
            pid2.SetMode(AUTOMATIC);
        }
        else
        {
            pid2.Compute();
            setMotor(M2_IN1, M2_IN2, M2_PWM, out2);
        }

        if (std::abs(sp3) < 10.0f)
        { // Threshold aumentado
            out3 = 0;
            setMotor(M3_IN1, M3_IN2, M3_PWM, 0);
            pid3.SetMode(MANUAL);
            pid3.SetMode(AUTOMATIC);
        }
        else
        {
            pid3.Compute();
            setMotor(M3_IN1, M3_IN2, M3_PWM, out3);
        }

        // Debug a cada 50 loops (~1 segundo)
        if (loopCount++ % 50 == 0)
        {
            std::cout << "M1: sp=" << (int)sp1 << " in=" << (int)in1 << " out=" << (int)out1
                      << " pos=" << enc1.getPosition() << std::endl;
            std::cout << "M2: sp=" << (int)sp2 << " in=" << (int)in2 << " out=" << (int)out2
                      << " pos=" << enc2.getPosition() << std::endl;
            std::cout << "M3: sp=" << (int)sp3 << " in=" << (int)in3 << " out=" << (int)out3
                      << " pos=" << enc3.getPosition() << std::endl;
            std::cout << "M4: speed=" << (int)enc4.getSpeed() 
                      << " pos=" << enc4.getPosition() 
                      << " (L1=" << c.l1 << " L2=" << c.l2 << ")" << std::endl;
            std::cout << "---" << std::endl;
        }

        gpioDelay(20000); // 20 ms
    }

    gpioTerminate();
    return 0;
}