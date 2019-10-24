import cv2
from tkinter import *


cap = cv2.VideoCapture(0)


while(True):
    # Capture frame-by-frame


    ret, frame = cap.read()

    # Our operations qon the frame come here
    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    cv2.imshow('output',grey)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    

# When everything done, release the capture
cv2.destroyAllWindows()