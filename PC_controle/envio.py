import pygame
import socket
import time

IP_RASP = "10.91.254.9"
PORTA = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

pygame.init()
pygame.joystick.init()

joy = pygame.joystick.Joystick(0)
joy.init()

clock = pygame.time.Clock()

while True:
    pygame.event.pump()
    
    lx = joy.get_axis(0)
    ly = joy.get_axis(1)
    rx = joy.get_axis(2)
    ry = joy.get_axis(3)
    
    msg = f"{lx:.3f},{ly:.3f},{rx:.3f},{ry:.3f}"
    sock.sendto(msg.encode(), (IP_RASP, PORTA))
    
    clock.tick(60)
