import cv2
import numpy as np
from matplotlib import pyplot as plt

params = cv2.SimpleBlobDetector_Params()

params.filterByColor = True
params.filterByArea = True
params.minArea = 100
params.filterByCircularity = False
params.filterByInertia = False
params.filterByConvexity = True



img = cv2.imread('dice_side.png',0)



blur = cv2.medianBlur(img, 5)



ret, binary_image = cv2.threshold(img, 50, 255, cv2.THRESH_BINARY)  # Binärer Schwellenwert anwenden

negativ = cv2.bitwise_not(binary_image)
cv2.imshow('schwellenwert', binary_image)
cv2.imshow('negativ', negativ)

kernel = np.ones((5,5),np.uint8)
erosion = cv2.erode(negativ,kernel,iterations = 1)

erosion = cv2.bitwise_not(erosion)

detector = cv2.SimpleBlobDetector_create(params)
keypoints = detector.detect(erosion)

img_with_keypoints = cv2.drawKeypoints(erosion, keypoints, np.array([]), (0, 0, 255),
                                       cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
cv2.imshow('keypoints', img_with_keypoints)

cv2.waitKey()