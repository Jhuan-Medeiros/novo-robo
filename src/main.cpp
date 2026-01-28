#include <iostream>
#include <pigpio.h>
#include <stdlib.h>
#include "../PC_controle/receive.h"
#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>
// ===== BIBLIOTECAS ADICIONAIS =====
#include "../modules/arduino/arduinoSerial.h"
#include "../modules/camera/cameraReceive.h"

// ================= PINOS =================

#define M1_IN1 27
#define M1_IN2 22
#define M1_PWM 17
#define M1_C1 23
#define M1_C2 24

#define M2_IN1 21
#define M2_IN2 20
#define M2_PWM 16
#define M2_C1 1
#define M2_C2 7

#define M3_IN1 11
#define M3_IN2 9
#define M3_PWM 10
#define M3_C1 25
#define M3_C2 8

#define M4_IN1 13
#define M4_IN2 6
#define M4_PWM 5
#define M4_C1 12
#define M4_C2 19

#define limite 4

// ================= PARAMETROS FÍSICOS =================

const float center = 0.103f;
const float radius = 0.033f;
const float sMax = 0.25f;
const float rMax = 1.0f;
const float CPR = 472.0f;
const float RAD_TO_PULSE = CPR / (2.0f * M_PI);

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

struct motor
{
    int in1, in2, pwmPin, c1, c2;

    volatile int32_t pos = 0;
    volatile uint8_t lastAB = 0;

    uint32_t prevTime = 0;
    int32_t prevPos = 0;
    float filteredSpeed = 0.0f;

    float integralError = 0.0f;

    float kp;
    float kFeed;
    int minPWM;
};

// ================= QUADRATURA =================

const int8_t quadTable[16] = {
    0, -1, +1, 0,
    +1, 0, 0, -1,
    -1, 0, 0, +1,
    0, +1, -1, 0};

// ================= INICIALIZAÇÃO DOS MOTORES =================

motor motors[4] = {
    {M1_IN1, M1_IN2, M1_PWM, M1_C1, M1_C2, 0, 0, 0, 0, 0.0f, 0.0f,
     0.2f, 0.5f, 35},

    {M2_IN1, M2_IN2, M2_PWM, M2_C1, M2_C2, 0, 0, 0, 0, 0.0f, 0.0f,
     0.2f, 0.6f, 50},

    {M3_IN1, M3_IN2, M3_PWM, M3_C1, M3_C2, 0, 0, 0, 0, 0.0f, 0.0f,
     0.2f, 0.5f, 35},

    {M4_IN1, M4_IN2, M4_PWM, M4_C1, M4_C2, 0, 0, 0, 0, 0.0f, 0.0f,
     0.2f, 0.5f, 0}};

void setupMotors(motor &m)
{
    gpioSetMode(m.in1, PI_OUTPUT);
    gpioSetMode(m.in2, PI_OUTPUT);
    gpioSetMode(m.pwmPin, PI_OUTPUT);

    gpioSetMode(m.c1, PI_INPUT);
    gpioSetMode(m.c2, PI_INPUT);
    gpioSetPullUpDown(m.c1, PI_PUD_UP);
    gpioSetPullUpDown(m.c2, PI_PUD_UP);

    gpioSetPWMfrequency(m.pwmPin, 1000);
    gpioSetPWMrange(m.pwmPin, 255);

    m.prevTime = gpioTick();
    m.prevPos = 0;
    m.filteredSpeed = 0;
    m.integralError = 0;
}

// ================= INTERRUPÇÃO (ISR) =================
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
void updateMotorSpeed(motor *m)
{
    uint32_t now = gpioTick();
    float dt = (now - m->prevTime) * 1e-6f;

    if (dt < 0.005f)
        return;

    int32_t currentPos = m->pos;
    int32_t delta = currentPos - m->prevPos;

    float rawSpeed = delta / dt;

    const float alpha = 0.2f;
    m->filteredSpeed = (rawSpeed * alpha) + (m->filteredSpeed * (1.0f - alpha));

    m->prevPos = currentPos;
    m->prevTime = now;
}

// ================= LÓGICA PID REFINADA =================

float computeHybridControl(motor *m, float targetSpeed)
{
    updateMotorSpeed(m);

    if (fabs(targetSpeed) < 2.0f)
    {
        m->integralError = 0;
        m->filteredSpeed = 0;
        return 0;
    }

    float feedforward = targetSpeed * m->kFeed;

    float error = targetSpeed - m->filteredSpeed;
    float pidOutput = (error * m->kp);

    float finalOutput = feedforward + pidOutput;

    int minForce = m->minPWM;

    if (finalOutput > 0 && finalOutput < minForce)
        finalOutput = minForce;
    if (finalOutput < 0 && finalOutput > -minForce)
        finalOutput = -minForce;

    return std::clamp(finalOutput, -255.0f, 255.0f);
}

// ================= CONTROLE DE MOTOR =================

void setMotorPWM(motor *m, float val)
{
    const int PWM_MIN = 35;

    int pwm = abs((int)val);

    if (pwm < PWM_MIN && pwm > 0)
    {
        pwm = 0;
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

void homing(motor &m, int &estado, bool sensorAtivo)
{
    if (estado == 0)
    {
        estado = 1;
    }

    switch (estado)
    {
    case 0:
        break;
    case 1:
        if (!sensorAtivo)
        {
            setMotorSimple(m.in1, m.in2, m.pwmPin, 255, false);
        }
        else
        {
            stopMotorSimple(m.in1, m.in2, m.pwmPin);
            m.pos = 0;
            std::cout << "Zero encontrado" << std::endl;
            gpioDelay(200000);
            estado = 2;
        }
    case 2:
        break;
    }
}

spdWheels conversion(float lx, float ly, float rx)
{
    float vx = lx * sMax;
    float vy = -ly * sMax;
    float w = rx * rMax;
    float headingOffset = 0.0f;

    float vx_r = vx * cos(headingOffset) - vy * sin(headingOffset);
    float vy_r = vx * sin(headingOffset) + vy * cos(headingOffset);

    vx_r = deadzone(vx_r, 0.01f);
    vy_r = deadzone(vy_r, 0.01f);
    w = deadzone(w, 0.01f);

    const float r = radius;
    const float L = center;

    const float th1 = 3.0f * M_PI / 2.0f;
    const float th2 = M_PI / 6.0f;
    const float th3 = 5.0f * M_PI / 6.0f;

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

    for (int i = 0; i < 4; i++)
    {
        setupMotors(motors[i]);

        uint8_t A = gpioRead(motors[i].c1);
        uint8_t B = gpioRead(motors[i].c2);
        motors[i].lastAB = (A << 1) | B;

        gpioSetAlertFuncEx(motors[i].c1, encoderISR, &motors[i]);
        gpioSetAlertFuncEx(motors[i].c2, encoderISR, &motors[i]);
    }

    gpioSetMode(limite, PI_INPUT);
    gpioSetPullUpDown(limite, PI_PUD_UP);

    int arduinoFd = configurarSerial("/dev/ttyACM0");
    if (arduinoFd == -1)
        std::cerr << "Erro Serial Arduino!" << std::endl;
    else
        std::cout << "Arduino OK!" << std::endl;

    iniciarRecepcaoControle();
    iniciarRecepcaoCamera();

    std::cout << "\n=== SISTEMA OMNI INTEGRADO COM VISÃO ===" << std::endl;
    std::cout << "Modo: PID Hibrido + Sensores + Pinça + CÂMERA" << std::endl;
    std::cout << "\nControles:" << std::endl;
    std::cout << "  - SHARE: Ativa/Desativa modo câmera" << std::endl;
    std::cout << "  - L1: Sobe elevador" << std::endl;
    std::cout << "  - L2: Desce elevador" << std::endl;
    std::cout << "  - TRIANGLE: Monitor de sensores" << std::endl;
    std::cout << "  - D-PAD: Controle dos servos\n"
              << std::endl;

    int loopCount = 0;
    int pinca_angulo = 50;
    int angulo_angulo = 0;

    int estadoElevador = 0;

    bool dLeftPressed = false, dRightPressed = false;
    bool dUpPressed = false, dDownPressed = false;

    // VARIÁVEIS PARA OS SENSORES
    bool trianglePressed = false;
    bool monitorSensor = false;

    // VARIÁVEIS PARA CÂMERA
    bool modoCamera = false;
    bool sharePressed = false;

    while (true)
    {
        controleState c = lerControle();
        CameraState cam = lerCamera();
        bool fimCursoAtivo = (gpioRead(limite) == PI_LOW);

        // ===== LÓGICA DO SHARE (MODO CÂMERA) =====
        if (c.share && !sharePressed)
        {
            modoCamera = !modoCamera;
            std::cout << "\n========================================" << std::endl;
            std::cout << "  🎥 MODO CÂMERA: " << (modoCamera ? "ATIVADO ✓" : "DESATIVADO ✗") << std::endl;
            std::cout << "========================================\n"
                      << std::endl;
            sharePressed = true;
        }
        else if (!c.share)
        {
            sharePressed = false;
        }

        // ===== LÓGICA DO TRIANGULO (SENSORES) =====
        if (c.triangle && !trianglePressed)
        {
            monitorSensor = !monitorSensor;
            std::cout << "\n=== MONITOR DE SENSORES "
                      << (monitorSensor ? "LIGADO" : "DESLIGADO")
                      << " ===\n"
                      << std::endl;
            trianglePressed = true;
        }
        else if (!c.triangle)
        {
            trianglePressed = false;
        }

        // ===== SERVOS (PINÇA: ABRIR/FECHAR) =====
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

        // ===== SERVOS (ÂNGULO: LEVANTAR/ABAIXAR) =====
        if (c.dUp && !dUpPressed)
        {
            angulo_angulo += SERVO_STEP;
            if (angulo_angulo > SERVO_2_MAX)
                angulo_angulo = SERVO_2_MAX;

            if (arduinoFd != -1)
                enviarServos(arduinoFd, pinca_angulo, angulo_angulo);
            dUpPressed = true;
        }
        else if (!c.dUp)
            dUpPressed = false;

        if (c.dDown && !dDownPressed)
        {
            angulo_angulo -= SERVO_STEP;
            if (angulo_angulo < SERVO_2_MIN)
                angulo_angulo = SERVO_2_MIN;

            if (arduinoFd != -1)
                enviarServos(arduinoFd, pinca_angulo, angulo_angulo);
            dDownPressed = true;
        }
        else if (!c.dDown)
            dDownPressed = false;

        // ===== MOTOR 4 (ELEVADOR) =====
        // L1 SOBE (forward=true) | L2 DESCE (forward=false)
        updateMotorSpeed(&motors[3]);

        homing(motors[3], estadoElevador, fimCursoAtivo);

        if (estadoElevador == 2)
        {
            if (c.l1)
            {
                // L1 SOBE
                setMotorSimple(M4_IN1, M4_IN2, M4_PWM, 255, true);
            }
            else if (c.l2 && !fimCursoAtivo)
            {
                // L2 DESCE (só se não estiver no fim de curso)
                setMotorSimple(M4_IN1, M4_IN2, M4_PWM, 255, false);
            }
            else
            {
                stopMotorSimple(M4_IN1, M4_IN2, M4_PWM);

                if (fimCursoAtivo)
                {
                    motors[3].pos = 0;
                }
            }
        }

        // ===== OMNI WHEELS =====
        spdWheels s;

        if (modoCamera)
        {
            float vx_auto = 0.0f;
            float rot_auto = 0.0f;

            // 🔍 DEBUG COMPLETO
            std::cout << "\n🔍 DEBUG CAMERA STATE:" << std::endl;
            std::cout << "   cam.comando = '" << cam.comando << "' (ASCII: " << (int)cam.comando << ")" << std::endl;
            std::cout << "   cam.erro_x = " << cam.erro_x << std::endl;
            std::cout << "   cam.distancia = " << cam.distancia << std::endl;
            std::cout << "   cam.alvo_detectado = " << cam.alvo_detectado << std::endl;

            if (cam.comando == 'D')
            {
                vx_auto = 0.6f;
                std::cout << "   ➡️ MOVENDO DIREITA (vx=0.6)" << std::endl;
            }
            else if (cam.comando == 'E')
            {
                vx_auto = -0.6f;
                std::cout << "   ⬅️ MOVENDO ESQUERDA (vx=-0.6)" << std::endl;
            }
            else if (cam.comando == 'P')
            {
                vx_auto = 0.0f;
                std::cout << "   ✅ PARADO (alinhado)" << std::endl;
            }
            else // cmd == 'N' ou qualquer outro
            {
                rot_auto = 0.3f;
                std::cout << "   🔄 PROCURANDO (rot=0.3, comando='" << cam.comando << "')" << std::endl;
            }

            s = conversion(vx_auto, 0.0f, rot_auto);

            std::cout << "   Targets: w1=" << (int)(s.w1 * RAD_TO_PULSE)
                      << " w2=" << (int)(s.w2 * RAD_TO_PULSE)
                      << " w3=" << (int)(s.w3 * RAD_TO_PULSE) << std::endl;
        }
        else
        {
            s = conversion(c.lx, c.ly, c.rx);
        }

        float out1 = computeHybridControl(&motors[0], s.w1 * RAD_TO_PULSE);
        float out2 = computeHybridControl(&motors[1], s.w2 * RAD_TO_PULSE);
        float out3 = computeHybridControl(&motors[2], s.w3 * RAD_TO_PULSE);

        setMotorPWM(&motors[0], out1);
        setMotorPWM(&motors[1], out2);
        setMotorPWM(&motors[2], out3);

        // ===== DEBUG / IMPRESSÃO =====

        if (loopCount++ % 50 == 0)
        {
            if (monitorSensor && arduinoFd != -1)
            {
                // MODO SENSORES: Mostra dados do Arduino
                std::string dados = lerSensores(arduinoFd);
                std::cout << "[SENSORES] " << dados << std::endl;
            }
            else
            {
                // MODO NORMAL: Mostra dados dos motores
                std::cout << "M1: Spd=" << (int)motors[0].filteredSpeed
                          << " Tgt=" << (int)(s.w1 * RAD_TO_PULSE)
                          << " PWM=" << (int)out1 << std::endl;
                std::cout << "M2: Spd=" << (int)motors[1].filteredSpeed
                          << " Tgt=" << (int)(s.w2 * RAD_TO_PULSE)
                          << " PWM=" << (int)out2 << std::endl;
                std::cout << "M3: Spd=" << (int)motors[2].filteredSpeed
                          << " Tgt=" << (int)(s.w3 * RAD_TO_PULSE)
                          << " PWM=" << (int)out3 << std::endl;
                std::cout << "M4 (Elev): Pos=" << motors[3].pos
                          << " FimCurso=" << (fimCursoAtivo ? "ATIVO" : "OFF") << std::endl;
                std::cout << "Servos: Pinca=" << pinca_angulo
                          << "° Angulo=" << angulo_angulo << "°" << std::endl;

                // Debug da câmera quando ativa
                if (modoCamera)
                {
                    std::cout << "📷 CAM: Cmd=" << cam.comando
                              << " | Erro=" << cam.erro_x << "px"
                              << " | Dist~" << cam.distancia << "cm"
                              << " | Alvo=" << (cam.alvo_detectado ? "✓" : "✗")
                              << std::endl;
                }

                std::cout << "---" << std::endl;
            }
        }

        gpioDelay(20000);
    }
}