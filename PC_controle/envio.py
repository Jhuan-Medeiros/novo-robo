import pygame
import socket
import time

IP_RASP = "10.91.238.188"
PORTA = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

pygame.init()
pygame.joystick.init()

joy = pygame.joystick.Joystick(0)
joy.init()

clock = pygame.time.Clock()

while True:
    pygame.event.pump()
    
    # Analógicos
    lx = joy.get_axis(0)
    ly = joy.get_axis(1)
    rx = joy.get_axis(2)
    ry = joy.get_axis(3)
    
    # D-pad
    dUp = joy.get_button(11)
    dDown = joy.get_button(12)
    dLeft = joy.get_button(13)
    dRight = joy.get_button(14)
    
    triangle = joy.get_button(3)
    
    # L1 - Left Bumper (botão digital simples)
    l1 = joy.get_button(9)
    
    # L2 - Left Trigger (Axis 4) - convertido para botão digital simples
    # Valores do eixo vão de -1.0 (solto) a 1.0 (pressionado)
    l2_axis = joy.get_axis(4)
    l2 = 1 if l2_axis > 0.1 else 0  # Se apertar > 10%, considera pressionado
    
    # SHARE - Botão 8 no PS4 (à esquerda do touchpad)
    share = joy.get_button(8)
    
    # Formato: lx,ly,rx,ry,dUp,dDown,dLeft,dRight,triangle,l1,l2,share
    msg = f"{lx:.3f},{ly:.3f},{rx:.3f},{ry:.3f},{dUp},{dDown},{dLeft},{dRight},{triangle},{l1},{l2},{share}"
    sock.sendto(msg.encode(), (IP_RASP, PORTA))
    
    clock.tick(60)