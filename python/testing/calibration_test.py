import cv2
import numpy as np
from tkinter import *
from matplotlib import pyplot as plt

root=Tk()

img = cv2.imread('dice.png',0)
blur = cv2.medianBlur(img, 5)
params = cv2.SimpleBlobDetector_Params()

params.filterByColor = True
params.filterByArea = True
params.minArea = 100
params.filterByCircularity = False
params.filterByInertia = False
params.filterByConvexity = True


def binary_trackbar(binary_value):
    ret, binary_image = cv2.threshold(blur, binary_value, 255, cv2.THRESH_BINARY)  # Binärer Schwellenwert anwenden
    
    detector = cv2.SimpleBlobDetector_create(params)
    keypoints = detector.detect(binary_image)

    img_with_keypoints = cv2.drawKeypoints(binary_image, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    cv2.imshow('TEST', img_with_keypoints)
    return keypoints
        
#def brightness(brightness_value):
 #   print(buffer)
 #   ret, binary_image = cv2.threshold(blur, buffer, 255, cv2.THRESH_BINARY)  # Binärer Schwellenwert anwenden
  #  cv2.imshow('TEST', binary_image)


def calibration():
    for i in range(256):
        keypoints = binary_trackbar(i)
        cv2.setTrackbarPos('Binary', 'TEST', i)
        cv2.waitKey(200)



        #print(img_with_keypoints.shape)
        #print(frame.shape)
        #print(grey.shape)
        #cv2.imshow("Press Q to close", img_with_keypoints);

        number = 0
        for x in keypoints[0:]:
            number = number + 1
        print(number)
        if number == 6:
            break


buffer=0

cv2.namedWindow('TEST')
cv2.createTrackbar('Binary', 'TEST', buffer, 255, binary_trackbar)
binary_trackbar(buffer)

edges = cv2.Canny(blur, 100, 200)
contours, hierarchy = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)



button1 = Button(root, text='kalibrieren', fg='red', command=calibration)
button1.pack()





#cv2.findContours(edges, mode, method[, contours[, hierarchy[, offset]]])



#cv2.imshow('TEST', img)

#cv2.imshow('TEST2', blur)


#numpy_horizontal_concat = np.concatenate((img, binary_image, edges), axis=1)
#cv2.imshow('Press Q to close this', numpy_horizontal_concat)

root.mainloop()

cv2.waitKey()