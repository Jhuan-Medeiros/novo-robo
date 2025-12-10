#include <iostream>
#include <pigpio.h>
#include "../PC_controle/receive.h"   // caminho relativo

int main()
{
    if (gpioInitialise() < 0) return 1;

    iniciarRecepcaoControle(); // começa a ouvir o controle

    while (true) {
        controleState c = lerControle(); // pega lx, ly, rx, ry

        // Exemplo de uso:
        std::cout << "LX=" << c.lx << " LY=" << c.ly
                  << " RX=" << c.rx << " RY=" << c.ry << std::endl;

        // aqui você usa c.lx, c.ly, etc. para controlar motor, etc.

        gpioDelay(20000); // 20ms
    }

    gpioTerminate();
    return 0;
}
