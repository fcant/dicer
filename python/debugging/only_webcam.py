import cv2
from tkinter import *
import numpy as np

cap = cv2.VideoCapture(0)


while(True):
    # Capture frame-by-frame


    ret, frame = cap.read()


    cv2.imshow('frame',frame)


    y = 160
    h = 240

    x = 220
    w = 240

    grey = frame
    
    real_image = frame[y:y + h, x:x + w]
    grey = cv2.cvtColor(real_image, cv2.COLOR_BGR2GRAY)
    
    

    y = 115
    h = 20

    pos_img = frame[y:y + h, x:x + w]
    pos_img = cv2.cvtColor(pos_img, cv2.COLOR_BGR2GRAY)
    ret, pos_img = cv2.threshold(pos_img, 245, 255, cv2.THRESH_BINARY)
    cv2.imshow('pos',pos_img)
    cv2.imshow('output',grey)

    M = cv2.moments(pos_img)  # Schwerpunkt berechnen

    


    if M["m00"] != 0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
    else:
        cX, cY = 0, 0

    print("X:", cX, "Y:", cY)
    ret, binary_image = cv2.threshold(grey, 230, 255,
                                      cv2.THRESH_BINARY)  # Schwellenwertbild abspeichern

    binary_image = cv2.bitwise_not(binary_image)

    # cv2.imshow('binary', binary_image)

    kernel_round = np.array([[0, 0, 0, 1, 1, 1, 0, 0, 0],
                             [0, 1, 1, 1, 1, 1, 1, 1, 0],
                             [0, 1, 1, 1, 1, 1, 1, 1, 0],
                             [1, 1, 1, 1, 1, 1, 1, 1, 1],
                             [1, 1, 1, 1, 1, 1, 1, 1, 1],
                             [1, 1, 1, 1, 1, 1, 1, 1, 1],
                             [0, 1, 1, 1, 1, 1, 1, 1, 0],
                             [0, 1, 1, 1, 1, 1, 1, 1, 0],
                             [0, 0, 0, 1, 1, 1, 0, 0, 0]], dtype=np.uint8)  # Kreisförmige Maske erzeugen

    dilate = cv2.dilate(binary_image, kernel_round, iterations=3)  # Dilatation anwenden

    erode = cv2.erode(dilate, kernel_round, iterations=2)  # Erosion anwenden

    cv2.imshow('binary', erode)

    img = cv2.medianBlur(erode, 5)  # Bild gätten mit Gauß
    cimg = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)  # Farbraum umwandeln (nur für die farbigen Kreise)
    circles = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT, 1, 20, param1=220, param2=10, minRadius=10,
                               maxRadius=30)  # param1: Schwellenwert, param2: muss man ausprobieren

    h_number = 0

    try:  # Kreise zählen und markieren
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            # draw the outer circle
            cv2.circle(cimg, (i[0], i[1]), i[2], (0, 255, 0), 2)
            # draw the center of the circle
            # cv2.circle(cimg, (i[0], i[1]), 3, (0, 255, 50), 4)
            h_number += 1
    except:
        print('HOUGH DETECTOR ERROR, NO CIRCLES FOUND')

    cv2.putText(cimg, str(h_number), (10, 200), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 50), 2, cv2.LINE_AA)
    cv2.imshow('hough detector - Press Q to exit', cimg)



    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    

# When everything done, release the capture
cv2.destroyAllWindows()