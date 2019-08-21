import cv2
import numpy as np

cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))

print(cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5)))

kernel = [[0,0,1,0,0],
          [0,1,1,1,0],
          [1,1,1,1,1],
          [0,1,1,1,0],
          [0,0,1,0,0]]
# np.ones((6, 6), np.uint8)

print(kernel)

