
import time
import numpy as np
import cv2

brightness = 0

cap = cv2.VideoCapture(0)
cv2.VideoCapture.set(cap, 10, brightness)

while(True):
    # Capture frame-by-frame


    ret, frame = cap.read()

    # Our operations on the frame come here
    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    y=200
    h=230
    
    x=260
    w=250

    grey = grey[y:y + h, x:x + w]

    cv2.imshow('raw',grey)

    cv2.imshow('output',grey)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()