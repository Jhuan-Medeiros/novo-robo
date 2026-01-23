#include <iostream>
#include <pigpio.h>
#include <stdlib.h>
#include "../PC_controle/receive.h"
#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>

// ===== BIBLIOTECAS ADICIONAIS =====
// Como não podemos usar bibliotecas externas, incluímos apenas o essencial
#include "../modules/arduino/arduinoSerial.h"

// ================= PINOS (CONFIGURAÇÃO DA OPÇÃO 1) =================

// Motor 1
#define M1_IN1 27
#define M1_IN2 22
#define M1_PWM 17
#define M1_C1 23
#define M1_C2 24

// Motor 2
#define M2_IN1 21
#define M2_IN2 20
#define M2_PWM 16
#define M2_C1 1
#define M2_C2 7

// Motor 3
#define M3_IN1 11
#define M3_IN2 9
#define M3_PWM 10
#define M3_C1 25
#define M3_C2 8

// Motor 4 (Elevador)
#define M4_IN1 13
#define M4_IN2 6
#define M4_PWM 5
#define M4_C1 12
#define M4_C2 19

// Fim de curso
#define limite 4

// ================= PARAMETROS FÍSICOS =================

const float center = 0.103f;
const float radius = 0.033f;
const float sMax = 0.30f;
const float rMax = 1.0f;
const float CPR = 472.0f; // Pulsos por rotação (Encoder físico)
const float RAD_TO_PULSE = CPR / (2.0f * M_PI);

// Servos
const int SERVO_1_MIN = 3;
const int SERVO_1_MAX = 55;
const int SERVO_2_MIN = 0;
const int SERVO_2_MAX = 60;
const int SERVO_STEP = 10;

// ================= ESTRUTURAS AVANÇADAS =================

struct spdWheels
{
    float w1;
    float w2;
    float w3;
};

// Estrutura remodelada para conter lógica de filtro e PID robusto
struct motor
{
    // Pinos
    int in1, in2, pwmPin, c1, c2;

    // Estado do Encoder
    volatile int32_t pos = 0;
    volatile uint8_t lastAB = 0;

    // Variáveis de Velocidade
    uint32_t prevTime = 0;
    int32_t prevPos = 0;
    float filteredSpeed = 0.0f; 

    float integralError = 0.0f;

    // --- CONFIGURAÇÕES INDIVIDUAIS ---
    float kp;      // Ganho de correção (PID)
    float kFeed;   // Ganho de acelerador (Feedforward)
    int minPWM;    // <--- O SEGREDO: Força mínima para a roda girar
};

// ================= QUADRATURA =================

const int8_t quadTable[16] = {
    0, -1, +1, 0,
    +1, 0, 0, -1,
    -1, 0, 0, +1,
    0, +1, -1, 0};

// ================= INICIALIZAÇÃO DOS MOTORES =================

// Inicialização: {PINOS...}, kp, kFeed, minPWM
motor motors[4] = {
    // Motor 1: Normal
    {M1_IN1, M1_IN2, M1_PWM, M1_C1, M1_C2, 0, 0, 0, 0, 0.0f, 0.0f, 
     0.2f, 0.5f, 35}, 

    // Motor 2: O "PREGUIÇOSO" -> Aumentei minPWM para 50 e kFeed para 0.6
    {M2_IN1, M2_IN2, M2_PWM, M2_C1, M2_C2, 0, 0, 0, 0, 0.0f, 0.0f, 
     0.2f, 0.6f, 50}, 

    // Motor 3: Normal
    {M3_IN1, M3_IN2, M3_PWM, M3_C1, M3_C2, 0, 0, 0, 0, 0.0f, 0.0f, 
     0.2f, 0.5f, 35}, 

    // Motor 4: Elevador (Configuração padrão, não usa PID híbrido msm)
    {M4_IN1, M4_IN2, M4_PWM, M4_C1, M4_C2, 0, 0, 0, 0, 0.0f, 0.0f, 
     0.2f, 0.5f, 0}
};

void setupMotors(motor &m)
{
    gpioSetMode(m.in1, PI_OUTPUT);
    gpioSetMode(m.in2, PI_OUTPUT);
    gpioSetMode(m.pwmPin, PI_OUTPUT);

    gpioSetMode(m.c1, PI_INPUT);
    gpioSetMode(m.c2, PI_INPUT);
    gpioSetPullUpDown(m.c1, PI_PUD_UP);
    gpioSetPullUpDown(m.c2, PI_PUD_UP);

    gpioSetPWMfrequency(m.pwmPin, 1000); // Frequência ajustada para motores DC comuns
    gpioSetPWMrange(m.pwmPin, 255);

    m.prevTime = gpioTick();
    m.prevPos = 0;
    m.filteredSpeed = 0;
    m.integralError = 0;
}

// ================= INTERRUPÇÃO (ISR) =================
// Mantida leve e rápida, apenas conta pulsos
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

// ================= FUNÇÃO DE CÁLCULO DE VELOCIDADE (COM FILTRO) =================
// Esta função é o segredo para parar a "metralhadora"
void updateMotorSpeed(motor *m)
{
    uint32_t now = gpioTick();
    float dt = (now - m->prevTime) * 1e-6f;

    // Proteção contra divisão por zero ou intervalos muito curtos
    if (dt < 0.005f)
        return;

    int32_t currentPos = m->pos;
    int32_t delta = currentPos - m->prevPos;

    // 1. Velocidade Crua (Ruidosa)
    float rawSpeed = delta / dt;

    // 2. Filtro de Média Móvel Exponencial (Low Pass Filter)
    // Alpha 0.2: Confia 20% na leitura nova, 80% no histórico.
    // Isso amortece os picos erráticos do encoder.
    const float alpha = 0.2f;
    m->filteredSpeed = (rawSpeed * alpha) + (m->filteredSpeed * (1.0f - alpha));

    // Atualiza histórico
    m->prevPos = currentPos;
    m->prevTime = now;
}

// ================= LÓGICA PID REFINADA =================

float computeHybridControl(motor *m, float targetSpeed)
{
    updateMotorSpeed(m); 

    // Zona Morta Lógica
    if (fabs(targetSpeed) < 2.0f)
    {
        m->integralError = 0;
        m->filteredSpeed = 0;
        return 0;
    }

    // 1. FEEDFORWARD (Acelerador)
    // Usa o kFeed específico deste motor
    float feedforward = targetSpeed * m->kFeed;

    // 2. PID SUAVE (Correção)
    float error = targetSpeed - m->filteredSpeed;
    float pidOutput = (error * m->kp); 

    float finalOutput = feedforward + pidOutput;

    // 3. BOOST DE ATRITO (INDIVIDUAL)
    // Usa o minPWM específico deste motor para vencer a inércia
    int minForce = m->minPWM;

    if (finalOutput > 0 && finalOutput < minForce) finalOutput = minForce;
    if (finalOutput < 0 && finalOutput > -minForce) finalOutput = -minForce;

    return std::clamp(finalOutput, -255.0f, 255.0f);
}

// ================= CONTROLE DE MOTOR =================

void setMotorPWM(motor *m, float val)
{
    // Zona morta de PWM: Motores geralmente não giram com PWM < 30
    const int PWM_MIN = 35;

    int pwm = abs((int)val);

    if (pwm < PWM_MIN && pwm > 0)
    {
        pwm = 0; // Se for muito fraco, corta energia para evitar zumbido
    }
    else if (pwm > 255)
    {
        pwm = 255;
    }

    if (val > 1.0f)
    {
        gpioWrite(m->in1, 1);
        gpioWrite(m->in2, 0);
    }
    else if (val < -1.0f)
    {
        gpioWrite(m->in1, 0);
        gpioWrite(m->in2, 1);
    }
    else
    {
        gpioWrite(m->in1, 0);
        gpioWrite(m->in2, 0);
        pwm = 0;
    }

    gpioPWM(m->pwmPin, pwm);
}

// Funções simples para o Motor 4 (Elevador)
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

// ================= AUXILIARES =================

float deadzone(float value, float zone = 0.20f)
{
    if (fabs(value) < zone)
        return 0;
    return value;
}

spdWheels conversion(float lx, float ly, float rx)
{
    float vx = lx * sMax;
    float vy = ly * sMax; // Removi o negativo, teste assim. Se inverter frente/trás, coloque o menos de volta (-ly)
    float w = rx * rMax;

    // === CORREÇÃO 1: Rotação Zerada ===
    // Se a frente do seu robô é onde estão as duas rodas (formato Y), deixe zero.
    // O -M_PI/4 (-45 graus) estava fazendo o robô andar torto.
    float headingOffset = 0.0f;

    float vx_r = vx * cos(headingOffset) - vy * sin(headingOffset);
    float vy_r = vx * sin(headingOffset) + vy * cos(headingOffset);

    // === CORREÇÃO 2: Deadzone Menor ===
    // 0.05 era muito alto para movimentos sutis. Baixei para 0.01.
    vx_r = deadzone(vx_r, 0.01f);
    vy_r = deadzone(vy_r, 0.01f);
    w = deadzone(w, 0.01f);

    const float r = radius;
    const float L = center;

    // Ângulos padrão para Omni 3 rodas (Formato Y)
    // Se a roda 1 estiver atrás: 270 graus (3*PI/2)
    // Roda 2 na frente esquerda: 30 graus (PI/6)
    // Roda 3 na frente direita: 150 graus (5*PI/6)
    // VERIFIQUE SE SEU ROBÔ ESTÁ MONTADO ASSIM
    const float th1 = 3.0f * M_PI / 2.0f; // Roda 1 (Atrás)
    const float th2 = M_PI / 6.0f;        // Roda 2 (Direita Frontal?)
    const float th3 = 5.0f * M_PI / 6.0f; // Roda 3 (Esquerda Frontal?)

    // Tentei reordenar os ângulos para o padrão mais comum.
    // Se as rodas girarem errado, troque os fios IN1/IN2.

    return {
        (-sin(th1) * vx_r + cos(th1) * vy_r + L * w) / r,
        (-sin(th2) * vx_r + cos(th2) * vy_r + L * w) / r,
        (-sin(th3) * vx_r + cos(th3) * vy_r + L * w) / r};
}

// ================= MAIN =================

int main()
{
    if (gpioInitialise() < 0)
        return 1;

    // Setup de todos os motores (incluindo M4)
    for (int i = 0; i < 4; i++)
    {
        setupMotors(motors[i]);

        // Leitura inicial para definir estado
        uint8_t A = gpioRead(motors[i].c1);
        uint8_t B = gpioRead(motors[i].c2);
        motors[i].lastAB = (A << 1) | B;

        gpioSetAlertFuncEx(motors[i].c1, encoderISR, &motors[i]);
        gpioSetAlertFuncEx(motors[i].c2, encoderISR, &motors[i]);
    }

    // Fim de curso
    gpioSetMode(limite, PI_INPUT);
    gpioSetPullUpDown(limite, PI_PUD_UP);

    // Arduino
    int arduinoFd = configurarSerial("/dev/ttyACM0");
    if (arduinoFd == -1)
        std::cerr << "Erro Serial Arduino!" << std::endl;
    else
        std::cout << "Arduino OK!" << std::endl;

    iniciarRecepcaoControle();

    std::cout << "\n=== SISTEMA OMNI INTEGRADO ===" << std::endl;
    std::cout << "Modo: PID Suavizado (Low Pass Filter)" << std::endl;

    int loopCount = 0;
    int pinca_angulo = 50;
    int angulo_angulo = 0;

    bool dLeftPressed = false, dRightPressed = false;
    bool dUpPressed = false, dDownPressed = false;

    while (true)
    {
        controleState c = lerControle();
        bool fimCursoAtivo = (gpioRead(limite) == PI_LOW);

        // ===== SERVOS (MANTER LÓGICA ORIGINAL) =====
        if (c.dLeft && !dLeftPressed)
        {
            pinca_angulo = SERVO_1_MIN;
            if (arduinoFd != -1)
                enviarServos(arduinoFd, pinca_angulo, angulo_angulo);
            dLeftPressed = true;
        }
        else if (!c.dLeft)
            dLeftPressed = false;

        if (c.dRight && !dRightPressed)
        {
            pinca_angulo = SERVO_1_MAX;
            if (arduinoFd != -1)
                enviarServos(arduinoFd, pinca_angulo, angulo_angulo);
            dRightPressed = true;
        }
        else if (!c.dRight)
            dRightPressed = false;

        // (Adicione aqui a lógica de dUp e dDown para o servo 2 igual ao original se precisar)

        // ===== MOTOR 4 (ELEVADOR) =====
        // M4 não usa PID, mas lemos o encoder para debug
        updateMotorSpeed(&motors[3]);

        if (c.l1)
        {
            setMotorSimple(M4_IN1, M4_IN2, M4_PWM, 255, true);
        }
        else if (c.l2 && !fimCursoAtivo)
        {
            setMotorSimple(M4_IN1, M4_IN2, M4_PWM, 255, false);
        }
        else
        {
            stopMotorSimple(M4_IN1, M4_IN2, M4_PWM);
        }

        // ===== MOVIMENTAÇÃO OMNI (MOTORES 1-3) =====

        spdWheels s = conversion(c.lx, c.ly, c.rx);

        // Calcula PID e aplica aos motores
        float out1 = computePID(&motors[0], s.w1 * RAD_TO_PULSE);
        float out2 = computePID(&motors[1], s.w2 * RAD_TO_PULSE);
        float out3 = computePID(&motors[2], s.w3 * RAD_TO_PULSE);

        setMotorPWM(&motors[0], out1);
        setMotorPWM(&motors[1], out2);
        setMotorPWM(&motors[2], out3);

        // ===== DEBUG =====
        if (loopCount++ % 50 == 0)
        {
            // Exibe a velocidade FILTRADA e a posição
            std::cout << "M1: Spd=" << (int)motors[0].filteredSpeed << " Tgt=" << (int)(s.w1 * RAD_TO_PULSE) << " PWM=" << (int)out1 << std::endl;
            std::cout << "M2: Spd=" << (int)motors[1].filteredSpeed << " Tgt=" << (int)(s.w2 * RAD_TO_PULSE) << " PWM=" << (int)out2 << std::endl;
            std::cout << "M3: Spd=" << (int)motors[2].filteredSpeed << " Tgt=" << (int)(s.w3 * RAD_TO_PULSE) << " PWM=" << (int)out3 << std::endl;
            std::cout << "M4 (Elev): Pos=" << motors[3].pos << " FimCurso=" << fimCursoAtivo << std::endl;
            std::cout << "---" << std::endl;
        }

        gpioDelay(20000); // 20ms Loop
    }

    gpioTerminate();
    return 0;
}