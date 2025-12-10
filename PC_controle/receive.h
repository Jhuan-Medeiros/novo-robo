#pragma once

struct controleState
{
    float lx;
    float ly;
    float rx;
    float ry;
};

void iniciarRecepcaoControle();

controleState lerControle();