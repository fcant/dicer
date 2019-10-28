import cv2
from tkinter import *


cap = cv2.VideoCapture(0)


while(True):
    # Capture frame-by-frame


    ret, frame = cap.read()

    y = 110
    h = 300

    x = 220
    w = 300

    real_image = frame[y:y + h, x:x + w]
    grey = cv2.cvtColor(real_image, cv2.COLOR_BGR2GRAY)

    y = 90
    h = 10

    pos_img = frame[y:y + h, x:x + w]
    pos_img = cv2.cvtColor(pos_img, cv2.COLOR_BGR2GRAY)
    ret, pos_img = cv2.threshold(pos_img, 245, 255, cv2.THRESH_BINARY)
    cv2.imshow('pos',pos_img)
    cv2.imshow('output',grey)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    

# When everything done, release the capture
cv2.destroyAllWindows()