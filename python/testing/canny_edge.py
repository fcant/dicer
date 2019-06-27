import cv2
import numpy as np
from matplotlib import pyplot as plt


img = cv2.imread('dark_dice.png',0)

empty = cv2.imread('empty.png',0) # >0: 3 channel, =0: grau, <0: bild + alpha

cv2.absdiff(img, empty, img)
q
cv2.imshow('DIFF', img)

blur = cv2.medianBlur(img, 5)

ret, binary_image = cv2.threshold(blur, 65, 255, cv2.THRESH_BINARY)

edges = cv2.Canny(binary_image, 100, 200)

contours, hierarchy = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

cnt = contours[0]

x,y,w,h = cv2.boundingRect(cnt)
cv2.rectangle(binary_image,(x,y),(x+w,y+h),(0,255,0),2)

img_crop = binary_image[y:y+h, x:x+w]

w = img_crop.shape[1] #y
h = img_crop.shape[0] #x

mask = np.zeros((h + 2, w + 2), np.uint8)

cv2.floodFill(img_crop, mask, (0,0), 255);

cv2.imshow('area', img_crop)


#cv2.drawContours(empty, contours, -1, (0,255,0), 3)

cv2.waitKey()