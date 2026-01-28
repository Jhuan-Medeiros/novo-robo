import cv2
import cv2.aruco as aruco
import numpy as np
import socket
import time

# ============= CONFIGURAÇÕES =============

ARUCO_ID_ALVO = 20

# LOCALHOST - Envia para o próprio Raspberry Pi
IP_LOCAL = "127.0.0.1"
PORTA_CAMERA = 5006

# Cria socket UDP
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Parâmetros da câmera
LARGURA_FRAME = 640
ALTURA_FRAME = 480
CENTRO_X = LARGURA_FRAME // 2
DEADZONE_X = 50

# ============= INICIALIZAÇÃO =============

print("Inicializando câmera no Raspberry Pi...")
camera = cv2.VideoCapture(0)  # /dev/video0

if not camera.isOpened():
    print("✗ ERRO: Não foi possível abrir a câmera!")
    exit()

camera.set(cv2.CAP_PROP_FRAME_WIDTH, LARGURA_FRAME)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, ALTURA_FRAME)
print("✓ Câmera inicializada")

arucoDic = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(arucoDic, parameters)

def enviar_comando_camera(comando, erro_x=0, distancia=0):
    """
    Envia comando via UDP para localhost (127.0.0.1)
    O C++ recebe na mesma máquina
    """
    msg = f"{comando},{erro_x},{distancia}"
    sock.sendto(msg.encode(), (IP_LOCAL, PORTA_CAMERA))
    return msg

# ============= LOOP PRINCIPAL =============

print(f"Procurando ArUco ID: {ARUCO_ID_ALVO}")
print(f"Enviando comandos para localhost:{PORTA_CAMERA}")
print("Pressione Ctrl+C para sair\n")

frame_count = 0

try:
    while True:
        ret, frame = camera.read()
        if not ret:
            print("✗ Erro ao capturar frame")
            time.sleep(0.1)
            continue

        cinza = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = detector.detectMarkers(cinza)
        
        comando_atual = "N"  # Nenhum alvo
        erro_x = 0
        distancia_estimada = 0

        if ids is not None:
            for i in range(len(ids)):
                if ids[i][0] == ARUCO_ID_ALVO:
                    c = corners[i][0]
                    x_marker = int(np.mean(c[:, 0]))
                    y_marker = int(np.mean(c[:, 1]))

                    # Estima distância baseado no tamanho do marcador
                    largura_marker = np.linalg.norm(c[0] - c[1])
                    distancia_estimada = int(1000 / (largura_marker + 1))

                    erro_x = x_marker - CENTRO_X

                    if abs(erro_x) < DEADZONE_X:
                        comando_atual = "P"
                        status = "CENTRALIZADO"
                    elif erro_x > 0:
                        comando_atual = "D"
                        status = "DIREITA"
                    else:
                        comando_atual = "E"
                        status = "ESQUERDA"
                    
                    break

        # Envia comando via UDP
        msg_enviada = enviar_comando_camera(comando_atual, erro_x, distancia_estimada)
        
        # Debug a cada 30 frames (não sobrecarrega o terminal)
        if frame_count % 30 == 0:
            if comando_atual != "N":
                print(f"→ {msg_enviada} ({status})")
            else:
                print("→ N,0,0 (Sem alvo)")
        
        frame_count += 1

        # Pequeno delay para não sobrecarregar a CPU
        time.sleep(0.033)  # ~30 FPS

except KeyboardInterrupt:
    print("\n✓ Encerrando câmera...")

finally:
    enviar_comando_camera("P", 0, 0)  # Garante parada
    sock.close()
    camera.release()
    print("✓ Câmera finalizada")