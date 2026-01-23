import cv2
import cv2.aruco as aruco
import numpy as np
import serial  # Biblioteca para comunicação USB
import time

# ============= CONFIGURAÇÕES =============

# ID do ArUco alvo
ARUCO_ID_ALVO = 1

PORTA_SERIAL = 'COM3'  # <--- ALTERE ISSO PARA SUA PORTA
BAUDRATE = 115200

# Parâmetros da câmera
LARGURA_FRAME = 640
ALTURA_FRAME = 480
CENTRO_X = LARGURA_FRAME // 2

# Tolerância (se estiver a menos de 50px do centro, considera centralizado)
DEADZONE_X = 50 

# ============= INICIALIZAÇÃO =============

# Inicializa Serial
try:
    ser = serial.Serial(PORTA_SERIAL, BAUDRATE, timeout=1)
    time.sleep(2) # Espera o Arduino/Microcontrolador reiniciar
    print(f"Conectado na porta {PORTA_SERIAL}")
except:
    print(f"ERRO: Não foi possível conectar na porta {PORTA_SERIAL}")
    print("Verifique se o cabo está conectado e se o nome da porta está correto.")
    exit()

# Câmera
camera = cv2.VideoCapture(0, cv2.CAP_V4L2) # Tente cv2.CAP_ANY se der erro
camera.set(cv2.CAP_PROP_FRAME_WIDTH, LARGURA_FRAME)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, ALTURA_FRAME)

# ArUco
arucoDic = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(arucoDic, parameters)

def enviar_comando_usb(comando):
    """Envia um caractere via USB"""
    if ser.is_open:
        # Envia a letra codificada em bytes (ex: b'E')
        ser.write(comando.encode())
        print(f"Enviado USB: {comando}")

# ============= LOOP PRINCIPAL =============

print(f"Procurando ArUco ID: {ARUCO_ID_ALVO}")
print("Pressione 'q' para sair")

try:
    while True:
        ret, frame = camera.read()
        if not ret: continue

        # Converte para cinza (melhora detecção)
        cinza = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detecta marcadores
        corners, ids, rejected = detector.detectMarkers(cinza)
        
        encontrou_alvo = False
        comando_atual = "P" # Padrão é Parar

        if ids is not None:
            # Desenha todos os marcadores para facilitar debug visual
            aruco.drawDetectedMarkers(frame, corners, ids)

            # Varre os IDs encontrados
            for i in range(len(ids)):
                if ids[i][0] == ARUCO_ID_ALVO:
                    encontrou_alvo = True
                    
                    # Pega os cantos do marcador atual
                    c = corners[i][0]
                    # Calcula o centro X do marcador (média dos 4 cantos)
                    x_marker = int(np.mean(c[:, 0]))
                    y_marker = int(np.mean(c[:, 1]))

                    # Desenha linha e ponto para visualização
                    cv2.circle(frame, (x_marker, y_marker), 5, (0, 0, 255), -1)
                    cv2.line(frame, (CENTRO_X, 0), (CENTRO_X, ALTURA_FRAME), (255, 0, 0), 2)

                    # --- LÓGICA DE CONTROLE SIMPLIFICADA ---
                    erro = x_marker - CENTRO_X

                    # Define o que fazer
                    if abs(erro) < DEADZONE_X:
                        comando_atual = "P" # Parar (Centralizado)
                        texto_status = "CENTRALIZADO"
                        cor = (0, 255, 0) # Verde
                    elif erro > 0:
                        comando_atual = "D" # Marker está na Direita -> Mover p/ Direita
                        texto_status = "MOVER DIREITA >>>"
                        cor = (0, 255, 255) # Amarelo
                    else:
                        comando_atual = "E" # Marker está na Esquerda -> Mover p/ Esquerda
                        texto_status = "<<< MOVER ESQUERDA"
                        cor = (0, 255, 255) # Amarelo

                    # Escreve na tela
                    cv2.putText(frame, f"Status: {texto_status}", (20, 40), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, cor, 2)
                    break 

        # Envia o comando via USB
        # (Se não encontrou nada, envia 'P' de Parar também)
        enviar_comando_usb(comando_atual)

        cv2.imshow("Rastreamento Simples", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    enviar_comando_usb("P") # Garante que o robô pare ao fechar
    ser.close()
    camera.release()
    cv2.destroyAllWindows()