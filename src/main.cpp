#include <iostream>
#include <pigpio.h>
#include <stdlib.h>
#include "../PC_controle/receive.h"
#define _USE_MATH_DEFINES
#include <cmath>

#define M1_IN1 27
#define M1_IN2 22
#define M1_PWM 17
#define M1_C1 23
#define M1_C2 24

#define M2_IN1 10
#define M2_IN2 9
#define M2_PWM 11
#define M2_C1 25
#define M2_C2 8

#define M3_IN1 5
#define M3_IN2 6
#define M3_PWM 13
#define M3_C1 7
#define M3_C2 1

const float center = 0.133f;
const float radius = 0.033f;

const float sMax = 0.30f;
const float rMax = 1.0f;

const float CPR = 472.0f;

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
    int c1;
    int c2;
    int pwmPin;
    int value;
    volatile long pos;

    volatile long prevPos = 0;
    uint32_t prevTime = 0;
    float prevError = 0;
    float integralError = 0;

    float kp = 0.6f;
    float ki = 0.5f;
    float kd = 0.05f;
};

motor motors[3] = {
    {M1_IN1, M1_IN2, M1_PWM, 0, M1_C1, M1_C2, 0},
    {M2_IN1, M2_IN2, M2_PWM, 0, M2_C1, M2_C2, 0},
    {M3_IN1, M3_IN2, M3_PWM, 0, M3_C1, M3_C2, 0}};

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

void encoderData(int gpio, int level, uint32_t tick, void *user) {
    if(level == 2) return;

    motor *m = (motor *)user;

    if(level != gpioRead(m->c2)) {
        m->pos ++;
    } else {
        m->pos --;
    }
}

float computePID(motor *m, float targetSpeed) {
    uint32_t currTime = gpioTick();

    float dt = (currTime - m->prevTime) / 1000000.0f;
    
    if(dt <= 0) return 0;

    float currentSpeed = (m->pos - m->prevPos) / dt;
    float error = targetSpeed - currentSpeed;

    float P = m->kp * error;

    m->integralError += error * dt;
    float I = m->ki * m->integralError;

    float D = m->kd * ((error - m->prevError) / dt);

    float output = P + I + D;

    m->prevPos = m->pos;
    m->prevTime = currTime;
    m-> prevError = error;

    if(std::abs(targetSpeed) < 2.0f) {
        m->prevError = 0;
        m->integralError = 0;
        m->prevTime = currTime;
        return 0;
    }

    if (dt <= 0.0001f) 
    {
        m->prevTime = currTime;
        return 0;
    }
    

    return output;
} 

float deadzone(float value, float zone = 0.20f)
{
    if (std::abs(value) < zone)
        return 0.0f;
    return value;
}

spdWheels conversion(float lx, float ly, float rx)
{
    // Controles
    float vx = ly * sMax;   // lateral
    float vy = lx * sMax;   // frente
    float w  = rx * rMax;   // rotação

    vx = deadzone(vx, 0.10f);
    vy = deadzone(vy, 0.10f);
    w  = deadzone(w, 0.10f);

    const float r = radius;
    const float L = center;

    // ÂNGULOS REAIS DAS RODAS (rad)
    const float th1 = 0.0f;                 // frente
    const float th2 = 2.0f * M_PI / 3.0f;    // 120°
    const float th3 = 4.0f * M_PI / 3.0f;    // 240°

    spdWheels spd;

    spd.w1 = (-sin(th1)*vx + cos(th1)*vy + L*w) / r;
    spd.w2 = (-sin(th2)*vx + cos(th2)*vy + L*w) / r;
    spd.w3 = (-sin(th3)*vx + cos(th3)*vy + L*w) / r;

    return spd;
}

void setMotor(int IN1, int IN2, int pwmPin, float controlValue)
{

    if (controlValue > 255) controlValue = 255;
    if(controlValue < -255) controlValue = -255;

    int pwmOutput = abs((int)controlValue);


    if (controlValue > 0.1f)
    {
        gpioWrite(IN1, PI_HIGH);
        gpioWrite(IN2, PI_LOW);
    }
    else if (controlValue < -0.1f)
    {
        gpioWrite(IN1, PI_LOW);
        gpioWrite(IN2, PI_HIGH);
    }
    else
    {
        gpioWrite(IN1, PI_LOW);
        gpioWrite(IN2, PI_LOW);
        pwmOutput = 0;
    }
    gpioPWM(pwmPin, pwmOutput);
}

int main()
{
    if (gpioInitialise() < 0)
        return 1;

    for (int i = 0; i < 3; i++)
    {
        setupMotors(motors[i]);
        gpioSetAlertFuncEx(motors[i].c1, encoderData, (void *)&motors[i]);
    }

    iniciarRecepcaoControle();

    const float RAD_TO_PULSE = CPR / (2.0f * M_PI);

    while (true)
    {
        controleState c = lerControle();

        float lx = c.lx;
        float ly = c.ly;
        float rx = c.rx;

        if (c.dUp == 1)
        {
            ly = 1.0f;
            lx = 0.0f;
            rx = 0.0f;
        }
        else if (c.dDown == 1)
        {
            ly = -1.0f;
            lx = 0.0f;
            rx = 0.0f;
        }

        if (c.dLeft == 1)
        {
            lx = -1.0f;
            ly = 0.0f;
            rx = 0.0f;
        }
        else if (c.dRight == 1)
        {
            lx = 1.0f;
            ly = 0.0f;
            rx = 0.0f;
        }

        spdWheels s = conversion(lx, ly, rx);

        float target1 = s.w1 * RAD_TO_PULSE;
        float pidOut1 = computePID(&motors[0], target1);
        setMotor(M1_IN1, M1_IN2, M1_PWM, pidOut1);
        
        float target2 = s.w2 * RAD_TO_PULSE;
        float pidOut2 = computePID(&motors[1], target2);
        setMotor(M2_IN1, M2_IN2, M2_PWM, pidOut2);
        
        float target3 = s.w3 * RAD_TO_PULSE;
        float pidOut3 = computePID(&motors[2], target3);
        setMotor(M3_IN1, M3_IN2, M3_PWM, pidOut3);
        

        gpioDelay(20000); // 20ms
    }

    gpioTerminate();
    return 0;
}
