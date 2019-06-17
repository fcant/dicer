import cv2

img1 = cv2.imread('empty.png',1)
img2 = cv2.imread('dice.png',1)

output = img2

cv2.imshow('output',output)

cv2.waitKey()