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
#include "../modules/arduino/arduinoSerial.h"

// ================= PINOS =================

#define M1_IN1 17
#define M1_IN2 27
#define M1_PWM 22

#define M2_IN1 11
#define M2_IN2 9
#define M2_PWM 10

#define M3_IN1 5
#define M3_IN2 6
#define M3_PWM 13

// Motor 4
#define M4_IN1 21
#define M4_IN2 20
#define M4_PWM 16

#define limite 4

// ================= PARAMETROS =================

const float center = 0.103f;
const float radius = 0.033f;

// Velocidades MUITO reduzidas para testes
const float sMax = 0.03f; // Velocidade bem reduzida
const float rMax = 0.05f; // Rotação bem lenta

const float CPR_PHYSICAL = 472.0f;
const float CPR_EFFECTIVE = CPR_PHYSICAL * 4.0f;

// Servos
const int SERVO_1_MIN = 3;
const int SERVO_1_MAX = 55;
const int SERVO_2_MIN = 0;
const int SERVO_2_MAX = 60;
const int SERVO_STEP = 10;

// ================= ESTRUTURAS =================

struct spdWheels
{
    float w1;
    float w2;
    float w3;
};

// ================= ENCODERS =================
// Agora usam índices 0-3 em vez de pinos GPIO

Encoder enc1(0);  // Motor 1 = Encoder index 0
Encoder enc2(1);  // Motor 2 = Encoder index 1
Encoder enc3(2);  // Motor 3 = Encoder index 2
Encoder enc4(3);  // Motor 4 = Encoder index 3

// ================= PID VARS =================

double in1 = 0, out1 = 0, sp1 = 0;
double in2 = 0, out2 = 0, sp2 = 0;
double in3 = 0, out3 = 0, sp3 = 0;

// ================= PID CONTROLLERS =================
// Ganhos aumentados para melhor resposta

PID pid1(&in1, &out1, &sp1, 1.0, 0.05, 0.0, DIRECT);
PID pid2(&in2, &out2, &sp2, 1.0, 0.05, 0.0, DIRECT);
PID pid3(&in3, &out3, &sp3, 1.0, 0.05, 0.0, DIRECT);

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
    float w = rx * rMax;

    vx = deadzone(vx, 0.03f);
    vy = deadzone(vy, 0.03f);
    w = deadzone(w, 0.03f);

    const float r = radius;
    const float L = center;

    spdWheels spd;

    // ===== CINEMÁTICA CORRETA PARA 3 RODAS OMNIDIRECIONAIS =====
    // Configuração física:
    //   - Roda 1: 270° (atrás, centralizada)
    //   - Roda 2: 30°  (frente-esquerda)
    //   - Roda 3: 150° (frente-direita)
    //
    // Fórmula: Wi = (-sin(α) * Vx + cos(α) * Vy + L * w) / r
    //
    // Simplificada com valores trigonométricos calculados:

    // Roda 1 (270°): sin(270°)=-1, cos(270°)=0
    spd.w1 = (vx + L * w) / r;
    
    // Roda 2 (30°): sin(30°)=0.5, cos(30°)=0.866
    spd.w2 = (-0.5f * vx + 0.866f * vy + L * w) / r;
    
    // Roda 3 (150°): sin(150°)=0.5, cos(150°)=-0.866
    spd.w3 = (-0.5f * vx - 0.866f * vy + L * w) / r;

    return spd;
}

void setMotor(int IN1, int IN2, int pwmPin, float controlValue)
{
    const int PWM_MIN = 30;

    if (controlValue > 255)
        controlValue = 255;
    if (controlValue < -255)
        controlValue = -255;

    int pwm = abs((int)controlValue);

    if (pwm > 0 && pwm < PWM_MIN)
    {
        pwm = PWM_MIN;
    }

    if (controlValue > 0)
    {
        gpioWrite(IN1, PI_HIGH);
        gpioWrite(IN2, PI_LOW);
    }
    else if (controlValue < 0)
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

// ================= TESTE DE MOTORES =================

void testarMotoresIndividualmente()
{
    std::cout << "\n=== TESTE DE MOTORES E ENCODERS ===" << std::endl;
    std::cout << "Este teste vai girar cada motor e verificar se o encoder" << std::endl;
    std::cout << "está lendo na direção correta (valor POSITIVO esperado)." << std::endl;
    std::cout << "\nPressione ENTER para iniciar..." << std::endl;
    std::cin.get();
    
    // Teste Motor 1
    std::cout << "\n--- Motor 1 ---" << std::endl;
    std::cout << "Girando motor em PWM +100..." << std::endl;
    enc1.reset();
    setMotor(M1_IN1, M1_IN2, M1_PWM, 100);
    gpioDelay(2000000); // 2 segundos
    float speed1 = enc1.getSpeed();
    long pos1 = enc1.getPosition();
    setMotor(M1_IN1, M1_IN2, M1_PWM, 0);
    std::cout << "Velocidade: " << speed1 << " pulsos/s" << std::endl;
    std::cout << "Posição: " << pos1 << " pulsos" << std::endl;
    if (speed1 < 0 || pos1 < 0) {
        std::cout << "⚠️  ATENÇÃO: Encoder NEGATIVO! Inverta os fios do Encoder 0 no Arduino Nano!" << std::endl;
    } else {
        std::cout << "✓ Motor 1 OK" << std::endl;
    }
    gpioDelay(1000000);
    
    // Teste Motor 2
    std::cout << "\n--- Motor 2 ---" << std::endl;
    std::cout << "Girando motor em PWM +100..." << std::endl;
    enc2.reset();
    setMotor(M2_IN1, M2_IN2, M2_PWM, 100);
    gpioDelay(2000000);
    float speed2 = enc2.getSpeed();
    long pos2 = enc2.getPosition();
    setMotor(M2_IN1, M2_IN2, M2_PWM, 0);
    std::cout << "Velocidade: " << speed2 << " pulsos/s" << std::endl;
    std::cout << "Posição: " << pos2 << " pulsos" << std::endl;
    if (speed2 < 0 || pos2 < 0) {
        std::cout << "⚠️  ATENÇÃO: Encoder NEGATIVO! Inverta os fios do Encoder 1 no Arduino Nano!" << std::endl;
    } else {
        std::cout << "✓ Motor 2 OK" << std::endl;
    }
    gpioDelay(1000000);
    
    // Teste Motor 3
    std::cout << "\n--- Motor 3 ---" << std::endl;
    std::cout << "Girando motor em PWM +100..." << std::endl;
    enc3.reset();
    setMotor(M3_IN1, M3_IN2, M3_PWM, 100);
    gpioDelay(2000000);
    float speed3 = enc3.getSpeed();
    long pos3 = enc3.getPosition();
    setMotor(M3_IN1, M3_IN2, M3_PWM, 0);
    std::cout << "Velocidade: " << speed3 << " pulsos/s" << std::endl;
    std::cout << "Posição: " << pos3 << " pulsos" << std::endl;
    if (speed3 < 0 || pos3 < 0) {
        std::cout << "⚠️  ATENÇÃO: Encoder NEGATIVO! Inverta os fios do Encoder 2 no Arduino Nano!" << std::endl;
    } else {
        std::cout << "✓ Motor 3 OK" << std::endl;
    }
    
    std::cout << "\n=== FIM DO TESTE ===" << std::endl;
    std::cout << "Pressione ENTER para iniciar o sistema normal..." << std::endl;
    std::cin.get();
}

// ================= MAIN =================

int main()
{
    if (gpioInitialise() < 0)
        return 1;

    // Configura apenas os pinos dos MOTORES (não mais os encoders)
    gpioSetMode(M1_IN1, PI_OUTPUT);
    gpioSetMode(M1_IN2, PI_OUTPUT);
    gpioSetMode(M1_PWM, PI_OUTPUT);

    gpioSetMode(M2_IN1, PI_OUTPUT);
    gpioSetMode(M2_IN2, PI_OUTPUT);
    gpioSetMode(M2_PWM, PI_OUTPUT);

    gpioSetMode(M3_IN1, PI_OUTPUT);
    gpioSetMode(M3_IN2, PI_OUTPUT);
    gpioSetMode(M3_PWM, PI_OUTPUT);

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

    gpioSetMode(limite, PI_INPUT);
    gpioSetPullUpDown(limite, PI_PUD_UP);

    // Arduino de sensores
    int arduinoFd = configurarSerial("/dev/ttyACM0");
    if (arduinoFd == -1)
    {
        std::cerr << "Erro ao abrir porta serial do Arduino de sensores!" << std::endl;
    }
    else
    {
        std::cout << "Arduino de sensores OK!" << std::endl;
    }

    // ===== INICIALIZA ARDUINO NANO DOS ENCODERS VIA UART =====
    std::cout << "\n=== INICIALIZANDO ARDUINO NANO (ENCODERS via UART) ===" << std::endl;
    
    // Tenta conectar na UART (TX/RX físico da Raspberry)
    // Portas comuns: /dev/serial0, /dev/ttyS0, /dev/ttyAMA0
    if (!Encoder::iniciarComunicacaoSerial("/dev/serial0")) {
        std::cout << "Tentativa /dev/serial0 falhou, tentando /dev/ttyS0..." << std::endl;
        if (!Encoder::iniciarComunicacaoSerial("/dev/ttyS0")) {
            std::cout << "Tentativa /dev/ttyS0 falhou, tentando /dev/ttyAMA0..." << std::endl;
            if (!Encoder::iniciarComunicacaoSerial("/dev/ttyAMA0")) {
                std::cerr << "\n❌ ERRO FATAL: Não foi possível conectar à UART do Arduino!" << std::endl;
                std::cerr << "\nVerifique:" << std::endl;
                std::cerr << "  1. UART habilitada no raspi-config (Interface Options -> Serial Port)" << std::endl;
                std::cerr << "     - Shell acessível via serial: NÃO" << std::endl;
                std::cerr << "     - Hardware serial habilitado: SIM" << std::endl;
                std::cerr << "  2. Conexões físicas:" << std::endl;
                std::cerr << "     Arduino TX (pino 1)  -> Raspberry RX (GPIO 15, pino 10)" << std::endl;
                std::cerr << "     Arduino RX (pino 0)  -> Raspberry TX (GPIO 14, pino 8)" << std::endl;
                std::cerr << "     Arduino GND          -> Raspberry GND" << std::endl;
                std::cerr << "  3. Arduino com código dos encoders carregado" << std::endl;
                std::cerr << "  4. Execute 'ls -l /dev/serial*' para verificar dispositivos disponíveis\n" << std::endl;
                
                gpioTerminate();
                return 1;
            }
        }
    }

    // Inicializa os encoders (apenas reseta variáveis internas)
    enc1.begin();
    enc2.begin();
    enc3.begin();
    enc4.begin();

    gpioDelay(100000);

    // ===== EXECUTAR TESTE DE MOTORES =====
    // Comente a linha abaixo após o primeiro teste
    testarMotoresIndividualmente();

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

    std::cout << "\n=== SISTEMA INICIADO ===" << std::endl;
    std::cout << "RAD_TO_PULSE: " << RAD_TO_PULSE << std::endl;
    std::cout << "Configuração:" << std::endl;
    std::cout << "  - Roda 1: Atrás (270°)" << std::endl;
    std::cout << "  - Roda 2: Frente-Esquerda (30°)" << std::endl;
    std::cout << "  - Roda 3: Frente-Direita (150°)" << std::endl;
    std::cout << "  - sMax: " << sMax << " m/s" << std::endl;
    std::cout << "  - rMax: " << rMax << " rad/s" << std::endl;
    std::cout << "========================\n" << std::endl;

    int loopCount = 0;

    int pinca_angulo = 50;
    int angulo_angulo = 0;

    bool dLeftPressed = false;
    bool dRightPressed = false;
    bool dUpPressed = false;
    bool dDownPressed = false;
    bool trianglePressed = false;

    bool monitorSensor = false;
    unsigned long lastPrint = 0;

    while (true)
    {
        controleState c = lerControle();

        float lx = c.lx;
        float ly = c.ly;
        float rx = c.rx;

        bool fimCursoAtivo = (gpioRead(limite) == PI_LOW);

        // Toggle monitor de sensores
        if (c.triangle && !trianglePressed)
        {
            monitorSensor = !monitorSensor;
            std::cout << "\n=== MONITOR DE SENSORES "
                      << (monitorSensor ? "LIGADO" : "DESLIGADO")
                      << " ===\n" << std::endl;
            trianglePressed = true;
        }
        else if (!c.triangle)
        {
            trianglePressed = false;
        }

        // Controle da pinça - Esquerda: Fechar
        if (c.dLeft && !dLeftPressed)
        {
            pinca_angulo = 3;
            if (arduinoFd != -1)
                enviarServos(arduinoFd, pinca_angulo, angulo_angulo);
            dLeftPressed = true;
        }
        else if (!c.dLeft)
        {
            dLeftPressed = false;
        }

        // Monitor de sensores
        if (monitorSensor && arduinoFd != -1)
        {
            unsigned long now = gpioTick();
            if (now - lastPrint >= 300000)
            {
                lastPrint = now;
                std::string dados = lerSensores(arduinoFd);
                std::cout << "\r[SENSORES] " << dados << "        " << std::flush;
            }
        }

        // Controle da pinça - Direita: Abrir
        if (c.dRight && !dRightPressed)
        {
            pinca_angulo = 50;
            if (arduinoFd != -1)
                enviarServos(arduinoFd, pinca_angulo, angulo_angulo);
            dRightPressed = true;
        }
        else if (!c.dRight)
        {
            dRightPressed = false;
        }

        // Controle do ângulo - Cima
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
        {
            dUpPressed = false;
        }

        // Controle do ângulo - Baixo
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
        {
            dDownPressed = false;
        }

        // Motor 4 (elevador)
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

        // Cinemática e controle PID
        spdWheels s = conversion(lx, ly, rx);

        sp1 = s.w1 * RAD_TO_PULSE;
        sp2 = s.w2 * RAD_TO_PULSE;
        sp3 = s.w3 * RAD_TO_PULSE;

        in1 = enc1.getSpeed();
        in2 = enc2.getSpeed();
        in3 = -enc3.getSpeed();

        // Motor 1
        if (std::abs(sp1) < 10.0f)
        {
            sp1 = 0;
            out1 = 0;
            setMotor(M1_IN1, M1_IN2, M1_PWM, 0);
        }
        else
        {
            pid1.Compute();
            setMotor(M1_IN1, M1_IN2, M1_PWM, out1);
        }

        // Motor 2
        if (std::abs(sp2) < 10.0f)
        {
            sp2 = 0;
            out2 = 0;
            setMotor(M2_IN1, M2_IN2, M2_PWM, 0);
        }
        else
        {
            pid2.Compute();
            setMotor(M2_IN1, M2_IN2, M2_PWM, out2);
        }

        // Motor 3
        if (std::abs(sp3) < 10.0f)
        {
            sp3 = 0;
            out3 = 0;
            setMotor(M3_IN1, M3_IN2, M3_PWM, 0);
        }
        else
        {
            pid3.Compute();
            setMotor(M3_IN1, M3_IN2, M3_PWM, out3);
        }

        // Debug a cada 50 loops
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
                      << " L1=" << c.l1 << " L2=" << c.l2
                      << " FimCurso=" << (fimCursoAtivo ? "ATIVO" : "inativo") << std::endl;
            std::cout << "Servos: S1=" << pinca_angulo << "° S2=" << angulo_angulo << "°" << std::endl;
            std::cout << "---" << std::endl;
        }

        gpioDelay(20000); // 20 ms
    }

    Encoder::pararComunicacaoSerial();
    gpioTerminate();
    return 0;
}