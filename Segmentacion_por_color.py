import cv2
import numpy as np
import os

# Cargar la imagen
image = cv2.imread(os.path.expanduser('~/Downloads/colors.jpg'))

# Convertir la imagen de BGR a HSV
hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Definir los rangos de color en HSV
lower_blue = np.array([100, 200, 50])
upper_blue = np.array([112, 255, 255])

# Crear una máscara utilizando el rango de color definido
mask = cv2.inRange(hsv_image, lower_blue, upper_blue)

# Aplicar la máscara a la imagen original
segmented_image = cv2.bitwise_and(image, image, mask=mask)

# Encontrar contornos en la máscara
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Contar las regiones segmentadas
count = len(contours)
print("Número de regiones segmentadas:", count)

# Dibujar los contornos en la imagen original
cv2.drawContours(image, contours, -1, (0, 255, 0), 2)

# Mostrar la imagen original y la imagen segmentada
# Mostrar la imagen con los contornos y el número de regiones segmentadas
cv2.putText(image, "Cantidad: " + str(count), (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)

0
cv2.imshow('Imagen Original | Programador: Gerardo Lee', image)
cv2.imshow('Segmentada | Programador: Gerardo Lee', segmented_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
