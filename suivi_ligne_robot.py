import cv2
import numpy as np
import serial
import time
from picamera2 import Picamera2
from libcamera import Transform

# Parametres
KP = 0.3
PWM_BASE = 60
PWM_MAX = 150  # Limite prudente pour les premiers essais

camera = Picamera2()
config = camera.create_preview_configuration(
    main={"size": (230, 120), "format": "RGB888"},
    transform=Transform(hflip=1, vflip=1), buffer_count=1
)
camera.configure(config)
camera.start()
time.sleep(1)
arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
time.sleep(2)
arduino.write(b'A10\n')  # Protocole ASCII avec acquittement
arduino.readline()


def moteurs(gauche, droite):
    gauche = int(np.clip(gauche, -PWM_MAX, PWM_MAX))
    droite = int(np.clip(droite, -PWM_MAX, PWM_MAX))
    # M1 avant droit, M2 avant gauche, M3 arriere droit, M4 arriere gauche
    arduino.write(f'C{droite} {gauche} {droite} {gauche}\n'.encode())
    arduino.readline()


try:

    while True:
        image = camera.capture_array()
        h, w = image.shape[:2]
        roi = image[int(h * 0.6):, :]  # 40 % inferieurs
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.blur(gray, (3, 3))
        _, mask = cv2.threshold(blur, 180, 255, cv2.THRESH_BINARY)
        kernel = np.ones((4, 4), np.uint8)
        mask = cv2.dilate(cv2.erode(mask, kernel), kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(contour)
            if cv2.contourArea(contour) > 50 and M['m00'] != 0:
                cx = M['m10'] / M['m00']
                erreur = cx - w / 2
                correction = KP * erreur
                moteurs(PWM_BASE + correction, PWM_BASE - correction)
            else:
                moteurs(0, 0)
        else:
            moteurs(0, 0)

#        cv2.imshow('Ligne', mask)
#        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    moteurs(0, 0)
    arduino.write(b'a\n')
    arduino.close()
    camera.stop()
    cv2.destroyAllWindows()
