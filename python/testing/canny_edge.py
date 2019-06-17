import cv2
import numpy as np
from matplotlib import pyplot as plt


#def trackbar_test(buffer):
#    print(buffer)


img = cv2.imread('dice.png',0)

blur = cv2.medianBlur(img, 5)


ret, binary_image = cv2.threshold(blur, 90, 255, cv2.THRESH_BINARY)  # Binärer Schwellenwert anwenden
edges = cv2.Canny(blur, 100, 200)


contours, hierarchy = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


#cv2.findContours(edges, mode, method[, contours[, hierarchy[, offset]]])

#cv2.namedWindow('TEST')
#buffer=0
#cv2.createTrackbar('Trackbar', 'TEST', buffer, 100,trackbar_test)
#trackbar_test(buffer)
#cv2.imshow('TEST', img)
#cv2.imshow('TEST2', blur)
#cv2.imshow('TEST', binary_image)

numpy_horizontal_concat = np.concatenate((img, binary_image, edges), axis=1)
cv2.imshow('Press Q to close this', numpy_horizontal_concat)

cv2.waitKey()