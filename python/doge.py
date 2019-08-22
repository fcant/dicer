import cv2
import sys
import numpy
numpy.set_printoptions(threshold=sys.maxsize)

doge = cv2.imread('dummy_image.png')

doge = cv2.cvtColor(doge, cv2.COLOR_BGR2GRAY)

file = open('doge', 'w')
file.write(str(doge))
file.close()

file = open('doge', 'r')
picture = file.read()
file.close()

print(picture)

cv2.imwrite('picture', picture)

cv2.waitKey()