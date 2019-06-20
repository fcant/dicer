import cv2
import numpy as np
from matplotlib import pyplot as plt


img = cv2.imread('dice_side.png',0)
blur = cv2.medianBlur(img, 5)
params = cv2.SimpleBlobDetector_Params()

params.filterByColor = True
params.filterByArea = True
params.minArea = 100
params.filterByCircularity = False
params.filterByInertia = False
params.filterByConvexity = True


def binary(binary_value):
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
  
    
    



edges = cv2.Canny(blur, 100, 200)
contours, hierarchy = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


#cv2.findContours(edges, mode, method[, contours[, hierarchy[, offset]]])

cv2.namedWindow('TEST')
buffer=0
cv2.createTrackbar('Trackbar', 'TEST', buffer, 255, binary)
binary(buffer)
x=0

if(x==1):
    for i in range(256):
        keypoints = binary(i)
        cv2.setTrackbarPos('Trackbar', 'TEST', i)
        cv2.waitKey(200)



        #print(img_with_keypoints.shape)
        #print(frame.shape)
        #print(grey.shape)
        #cv2.imshow("Press Q to close", img_with_keypoints);

        number = 0
        for i in keypoints[0:]:
            number = number + 1
        print(number)
        if number == 6:
            break




#cv2.imshow('TEST', img)

#cv2.imshow('TEST2', blur)


#numpy_horizontal_concat = np.concatenate((img, binary_image, edges), axis=1)
#cv2.imshow('Press Q to close this', numpy_horizontal_concat)

cv2.waitKey()