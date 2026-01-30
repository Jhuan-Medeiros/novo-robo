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
#define M2_C1 7
#define M2_C2 1

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
const float sMax = 0.20f;
const float rMax = 1.0f;
const float CPR = 472.0f;
const float RAD_TO_PULSE = CPR / (2.0f * M_PI);

const int SERVO_1_MIN = 3;  // Pinça aberta
const int SERVO_1_MAX = 55; // Pinça fechada
const int SERVO_2_MIN = 0;  // Ângulo mínimo (baixo)
const int SERVO_2_MAX = 60; // Ângulo máximo (alto)
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

// ================= ESTADOS DA SEQUÊNCIA AUTOMÁTICA =================
enum EstadoSequencia
{
    SEQ_IDLE = 0,         // Aguardando
    SEQ_ALINHANDO,        // Alinhando com ArUco
    SEQ_SUBINDO_120,     // Subindo elevador 3000 passos
    SEQ_ABRINDO_GARRA,    // Abrindo garra
    SEQ_FECHANDO_GARRA,   // Fechando garra
    SEQ_LEVANTANDO_GARRA, // Levantando garra ao máximo
    SEQ_AGUARDANDO_3S,    // Esperando 3 segundos
    SEQ_SUBINDO_7880,     // Subindo mais 5000 passos
    SEQ_DESCENDO_GARRA,   // Descendo garra ao mínimo
    SEQ_ABRINDO_FINAL,    // Abrindo garra final
    SEQ_COMPLETO          // Sequência completa
};

// ================= QUADRATURA =================

const int8_t quadTable[16] = {
    0, -1, +1, 0,
    +1, 0, 0, -1,
    -1, 0, 0, +1,
    0, +1, -1, 0};

// ================= INICIALIZAÇÃO DOS MOTORES =================
motor motors[4] = {
    {M1_IN1, M1_IN2, M1_PWM, M1_C1, M1_C2, 0, 0, 0, 0, 0.0f, 0.0f, 0.8f, 1.5f, 80},
    {M2_IN1, M2_IN2, M2_PWM, M2_C1, M2_C2, 0, 0, 0, 0, 0.0f, 0.0f, 0.18f, 0.8f, 45},
    {M3_IN1, M3_IN2, M3_PWM, M3_C1, M3_C2, 0, 0, 0, 0, 0.0f, 0.0f, 0.18f, 0.8f, 45},
    {M4_IN1, M4_IN2, M4_PWM, M4_C1, M4_C2, 0, 0, 0, 0, 0.0f, 0.0f, 0.2f, 0.5f, 0}};

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

static bool g_modoCameraAtivo = false;

void setModoCameraAtivo(bool ativo)
{
    g_modoCameraAtivo = ativo;
}

void setMotorPWM(motor *m, float val)
{
    int pwm = abs((int)val);

    if (g_modoCameraAtivo)
    {
        const int PWM_MIN_CAMERA = 120;
        if (pwm > 0 && pwm < PWM_MIN_CAMERA)
        {
            pwm = PWM_MIN_CAMERA;
        }
        else if (pwm > 255)
        {
            pwm = 255;
        }
    }
    else
    {
        const int PWM_MIN = 35;
        if (pwm < PWM_MIN && pwm > 0)
        {
            pwm = 0;
        }
        else if (pwm > 255)
        {
            pwm = 255;
        }
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

    vx_r = deadzone(vx_r, 0.10f);
    vy_r = deadzone(vy_r, 0.10f);
    w = deadzone(w, 0.10f);

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

    std::cout << "\n=== SISTEMA OMNI COM SEQUÊNCIA AUTOMÁTICA ===" << std::endl;
    std::cout << "Modo: Detecção ArUco + Sequência Automatizada" << std::endl;
    std::cout << "\nControles:" << std::endl;
    std::cout << "  - SHARE: Ativa modo câmera (busca ArUco)" << std::endl;
    std::cout << "  - Ao centralizar ArUco: executa sequência automática" << std::endl;
    std::cout << "  - L1: Sobe elevador (manual)" << std::endl;
    std::cout << "  - L2: Desce elevador (manual)" << std::endl;
    std::cout << "  - TRIANGLE: Monitor de sensores" << std::endl;
    std::cout << "  - D-PAD: Controle dos servos (manual)\n"
              << std::endl;

    int loopCount = 0;
    int pinca_angulo = 50;
    int angulo_angulo = 0;

    int estadoElevador = 0;

    bool dLeftPressed = false, dRightPressed = false;
    bool dUpPressed = false, dDownPressed = false;

    bool trianglePressed = false;
    bool monitorSensor = false;

    bool modoCamera = false;
    bool sharePressed = false;

    // VARIÁVEIS DA SEQUÊNCIA AUTOMÁTICA
    EstadoSequencia estadoSequencia = SEQ_IDLE;
    int32_t posicaoInicial = 0;
    int32_t posicaoAlvo = 0;
    uint32_t tempoInicio = 0;
    int contadorCentralizado = 0; // Conta frames centralizados

    while (true)
    {
        controleState c = lerControle();
        CameraState cam = lerCamera();
        bool fimCursoAtivo = (gpioRead(limite) == PI_LOW);

        // ===== LÓGICA DO SHARE (MODO CÂMERA) =====
        if (c.share && !sharePressed)
        {
            modoCamera = !modoCamera;
            setModoCameraAtivo(modoCamera);

            if (modoCamera)
            {
                estadoSequencia = SEQ_ALINHANDO; // Inicia sequência
            }
            else
            {
                estadoSequencia = SEQ_IDLE; // Cancela sequência
            }

            std::cout << "\n========================================" << std::endl;
            std::cout << "  🎥 MODO CÂMERA: " << (modoCamera ? "ATIVADO ✓" : "DESATIVADO ✗") << std::endl;
            if (modoCamera)
            {
                std::cout << "  Iniciando busca por ArUco..." << std::endl;
            }
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

        // ===== CONTROLE MANUAL DOS SERVOS (se não estiver em sequência) =====
        if (estadoSequencia == SEQ_IDLE || estadoSequencia == SEQ_COMPLETO)
        {
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
        }

        // ===== MOTOR 4 (ELEVADOR) - HOMING =====
        updateMotorSpeed(&motors[3]);
        homing(motors[3], estadoElevador, fimCursoAtivo);

        // ===== SEQUÊNCIA AUTOMÁTICA =====
        if (estadoElevador == 2) // Só executa após homing
        {
            switch (estadoSequencia)
            {
            case SEQ_IDLE:
            case SEQ_COMPLETO:
                // Controle manual do elevador
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
                    if (fimCursoAtivo)
                    {
                        motors[3].pos = 0;
                    }
                }
                break;

            case SEQ_ALINHANDO:
                if (!modoCamera)
                { // segurança
                    estadoSequencia = SEQ_IDLE;
                    break;
                }

                if (cam.comando == 'P')
                {
                    contadorCentralizado++;
                    if (contadorCentralizado > 30)
                    {
                        std::cout << "\n✓ ArUco centralizado! Iniciando sequência...\n"
                                  << std::endl;

                        posicaoInicial = motors[3].pos;
                        posicaoAlvo = posicaoInicial + 120;

                        estadoSequencia = SEQ_SUBINDO_120;
                        contadorCentralizado = 0;


                    }
                }
                else
                {
                    contadorCentralizado = 0;
                }
                break;

            case SEQ_SUBINDO_120:
                std::cout << "[SEQ] Subindo 120 passos... (" << motors[3].pos << "/" << posicaoAlvo << ")" << std::endl;
                if (motors[3].pos < posicaoAlvo)
                {
                    setMotorSimple(M4_IN1, M4_IN2, M4_PWM, 200, true);
                }
                else
                {
                    stopMotorSimple(M4_IN1, M4_IN2, M4_PWM);
                    std::cout << "✓ 120 passos completos!\n"
                              << std::endl;
                    estadoSequencia = SEQ_ABRINDO_GARRA;
                    gpioDelay(500000); // 0.5s de pausa
                }
                break;

            case SEQ_ABRINDO_GARRA:
                std::cout << "[SEQ] Abrindo garra..." << std::endl;
                pinca_angulo = SERVO_1_MIN; // Abre
                if (arduinoFd != -1)
                    enviarServos(arduinoFd, pinca_angulo, angulo_angulo);
                gpioDelay(1000000); // 1s
                std::cout << "✓ Garra aberta!\n"
                          << std::endl;
                estadoSequencia = SEQ_FECHANDO_GARRA;
                break;

            case SEQ_FECHANDO_GARRA:
                std::cout << "[SEQ] Fechando garra..." << std::endl;
                pinca_angulo = SERVO_1_MAX; // Fecha
                if (arduinoFd != -1)
                    enviarServos(arduinoFd, pinca_angulo, angulo_angulo);
                gpioDelay(1000000); // 1s
                std::cout << "✓ Garra fechada!\n"
                          << std::endl;
                estadoSequencia = SEQ_LEVANTANDO_GARRA;
                break;

            case SEQ_LEVANTANDO_GARRA:
                std::cout << "[SEQ] Levantando garra ao máximo..." << std::endl;
                angulo_angulo = SERVO_2_MAX; // Levanta
                if (arduinoFd != -1)
                    enviarServos(arduinoFd, pinca_angulo, angulo_angulo);
                gpioDelay(1500000); // 1.5s
                std::cout << "✓ Garra levantada!\n"
                          << std::endl;
                tempoInicio = gpioTick();
                estadoSequencia = SEQ_AGUARDANDO_3S;
                break;

            case SEQ_AGUARDANDO_3S:
            {
                uint32_t tempoDecorrido = (gpioTick() - tempoInicio) / 1000; // ms
                if (tempoDecorrido >= 3000) // 3 segundos
                {
                    std::cout << "✓ 3 segundos completos!\n"
                              << std::endl;
                    posicaoInicial = motors[3].pos;
                    posicaoAlvo = posicaoInicial + 7880;
                    estadoSequencia = SEQ_SUBINDO_7880;
                }
                if (loopCount % 50 == 0)
                {
                    std::cout << "[SEQ] Aguardando... " << tempoDecorrido / 1000 << "/3s" << std::endl;
                }
            }
            break;

            case SEQ_SUBINDO_7880:
                std::cout << "[SEQ] Subindo mais 7880 passos... (" << motors[3].pos << "/" << posicaoAlvo << ")" << std::endl;
                if (motors[3].pos < posicaoAlvo)
                {
                    setMotorSimple(M4_IN1, M4_IN2, M4_PWM, 200, true);
                }
                else
                {
                    stopMotorSimple(M4_IN1, M4_IN2, M4_PWM);
                    std::cout << "✓ 7880 passos completos!\n"
                              << std::endl;
                    estadoSequencia = SEQ_DESCENDO_GARRA;
                    gpioDelay(500000);
                }
                break;

            case SEQ_DESCENDO_GARRA:
                std::cout << "[SEQ] Descendo garra ao mínimo..." << std::endl;
                angulo_angulo = SERVO_2_MIN; // Desce
                if (arduinoFd != -1)
                    enviarServos(arduinoFd, pinca_angulo, angulo_angulo);
                gpioDelay(1500000); // 1.5s
                std::cout << "✓ Garra descida!\n"
                          << std::endl;
                estadoSequencia = SEQ_ABRINDO_FINAL;
                break;

            case SEQ_ABRINDO_FINAL:
                std::cout << "[SEQ] Abrindo garra final..." << std::endl;
                pinca_angulo = SERVO_1_MIN; // Abre
                if (arduinoFd != -1)
                    enviarServos(arduinoFd, pinca_angulo, angulo_angulo);
                gpioDelay(1000000); // 1s
                std::cout << "\n🎉 ===== SEQUÊNCIA COMPLETA! =====\n"
                          << std::endl;
                estadoSequencia = SEQ_COMPLETO;
                modoCamera = false; // Desativa modo câmera
                setModoCameraAtivo(false);
                break;
            }
        }

        // ===== OMNI WHEELS =====
        spdWheels s;

        if (modoCamera)
        {
            s = conversion(0.0f, 0.0f, 0.0f);
        }
        else
        {
            if (estadoSequencia == SEQ_IDLE || estadoSequencia == SEQ_COMPLETO)
            {
                s = conversion(c.lx, c.ly, c.rx);
            }
            else
            {
                s = conversion(0.0f, 0.0f, 0.0f);
            }
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
                std::string dados = lerSensores(arduinoFd);
                std::cout << "[SENSORES] " << dados << std::endl;
            }
            else if (estadoSequencia == SEQ_IDLE || estadoSequencia == SEQ_COMPLETO)
            {
                // Debug normal apenas quando inativo
                std::cout << "M4 (Elev): Pos=" << motors[3].pos
                          << " | Servos: Pinca=" << pinca_angulo
                          << "° Angulo=" << angulo_angulo << "°" << std::endl;

                if (modoCamera)
                {
                    std::cout << "📷 CAM: Cmd=" << cam.comando
                              << " | Erro=" << cam.erro_x << "px" << std::endl;
                }
                std::cout << "---" << std::endl;
            }
        }

        gpioDelay(20000);
    }
}