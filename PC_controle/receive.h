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

    int triangle;
    int l1;
    int l2;
};

void iniciarRecepcaoControle();

controleState lerControle();
