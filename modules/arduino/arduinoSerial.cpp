#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <string>

int configurarSerial(const char* porta) {
    int fd = open(porta, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) return -1;

    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    tcsetattr(fd, TCSANOW, &options);
    return fd;
}

void enviarServos(int fd, int pinca, int angulo) {
    std::string comando = std::to_string(pinca) + "," + std::to_string(angulo) + "\n";
    write(fd, comando.c_str(), comando.length());
}