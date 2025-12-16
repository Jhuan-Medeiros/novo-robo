#pragma once

struct controleState
{
    float lx;
    float ly;
    float rx;
    float ry;

    int dUp;
    int dDown;
    int dLeft;
    int dRight;
};

void iniciarRecepcaoControle();

controleState lerControle();