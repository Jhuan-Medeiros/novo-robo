#pragma once
#include <string>

struct CameraState
{
    char comando;
    int erro_x;
    int distancia;
    bool alvo_detectado;
};

void iniciarRecepcaoCamera();

CameraState lerCamera();