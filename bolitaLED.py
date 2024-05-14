import serial
import time
from picamera2 import Picamera2
import cv2
import numpy as np

# Inicializar la cámara
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480)}))
picam2.start()

# Inicializar conexión serial con Arduino
arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
time.sleep(2)  # Esperar a que Arduino se inicialice

def process_frame(frame):
    # Convertir a escala de grises
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Aplicar un filtro de desenfoque
    blurred = cv2.GaussianBlur(gray, (11, 11), 0)
    
    # Detectar bordes usando Canny
    edges = cv2.Canny(blurred, 30, 150)
    
    # Usar la transformación de Hough para detectar círculos
    circles = cv2.HoughCircles(edges, cv2.HOUGH_GRADIENT, dp=1.2, minDist=50, param1=100, param2=30, minRadius=10, maxRadius=100)
    
    # Verificar si se han detectado círculos
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        
        for (x, y, r) in circles:
            # Calcular el centro de la bola
            ball_center = (x, y)
            
            # Encender LED correspondiente en base a la posición de la bola
            if ball_center[0] < 320:  # Bola a la izquierda
                arduino.write(b'7')
                print("Encendiendo LED 7")
            elif ball_center[0] > 320:  # Bola a la derecha
                arduino.write(b'13')
                print("Encendiendo LED 13")
            else:  # Bola en el centro
                arduino.write(b'713')
                print("Encendiendo LED 7 y 13")
            
            # Extraer el ROI de la bola
            roi = frame[y-r:y+r, x-r:x+r]
            if roi.size == 0:
                continue
            
            # Convertir el ROI a HSV para analizar el color
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            
            # Definir rangos de color para la bola plateada y negra
            lower_silver = np.array([0, 0, 120])
            upper_silver = np.array([180, 30, 255])
            lower_black = np.array([0, 0, 0])
            upper_black = np.array([180, 255, 50])
            
            # Crear máscaras para los colores
            mask_silver = cv2.inRange(hsv, lower_silver, upper_silver)
            mask_black = cv2.inRange(hsv, lower_black, upper_black)
            
            # Contar píxeles no negros en las máscaras
            silver_pixels = cv2.countNonZero(mask_silver)
            black_pixels = cv2.countNonZero(mask_black)
            
            # Definir umbral para considerar una detección válida
            if silver_pixels > 0.5 * (roi.shape[0] * roi.shape[1]):
                cv2.circle(frame, (x, y), r, (0, 255, 0), 2)
                cv2.circle(frame, (x, y), 5, (0, 128, 255), -1)
                print(f'Bola Plateada Centro: ({x}, {y}), Diámetro: {2*r}')
            elif black_pixels > 0.5 * (roi.shape[0] * roi.shape[1]):
                cv2.circle(frame, (x, y), r, (255, 0, 0), 2)
                cv2.circle(frame, (x, y), 5, (0, 128, 255), -1)
                print(f'Bola Negra Centro: ({x}, {y}), Diámetro: {2*r}')
    
    return frame

# Bucle principal para capturar y procesar cuadros en tiempo real
while True:
    # Capturar el cuadro actual
    frame = picam2.capture_array()
    
    # Procesar el cuadro
    processed_frame = process_frame(frame)
    
    # Mostrar el cuadro procesado
    cv2.imshow('Detected Balls', processed_frame)
    
    # Salir del bucle si se presiona la tecla 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Liberar los recursos
cv2.destroyAllWindows()
picam2.stop()
arduino.close()
