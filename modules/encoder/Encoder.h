#pragma once
#include <atomic>
#include <cstdint>
#include <thread>

class Encoder {
public:
    Encoder(int index);  // index: 0-3 para identificar qual encoder (M1, M2, M3, M4)

    void begin();  // Inicializa (mas a leitura serial é compartilhada)
    
    long  getPosition();
    float getSpeed();
    void  reset();

    static bool iniciarComunicacaoSerial(const char* porta);
    static void pararComunicacaoSerial();

private:
    int encoderIndex;  // 0, 1, 2 ou 3
    
    std::atomic<long> position;
    
    uint32_t lastTime;
    long lastPos;
    float lastSpeed;

    static int serialFd;
    static std::atomic<bool> threadAtiva;
    static std::atomic<int32_t> positionsShared[4];  // Posições lidas do Arduino
    
    static void threadLeituraSerial();
    static uint32_t getMicros();
};