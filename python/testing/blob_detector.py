import cv2
import numpy as np


params = cv2.SimpleBlobDetector_Params()
print (params.filterByColor)
print (params.filterByArea)
print (params.filterByCircularity)
print (params.filterByInertia)
print (params.filterByConvexity)

image = cv2.imread('shape.png',1) # >0: 3 channel, =0: grau, <0: bild + alpha

params.filterByInertia = False
params.filterByConvexity = False

detector = cv2.SimpleBlobDetector_create(params)
print (params.filterByColor)
print (params.filterByArea)
print (params.filterByCircularity)
print (params.filterByInertia)
print (params.filterByConvexity)

keypoints = detector.detect(image)

img_with_keypoints = cv2.drawKeypoints(image, keypoints, np.array([]), (0, 0, 255),cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

cv2.imshow("keypoints", img_with_keypoints);
cv2.waitKey()

number = 0
for i in keypoints[0:]:
    number = number + 1

print ('Number of blobs:',number)


# numpy_horizontal_concat = np.concatenate((grey, output, img_with_keypoints), axis=1)
# cv2.imshow('Press Q to close this', numpy_horizontal_concat)