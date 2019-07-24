import cv2
from tkinter import *


cap = cv2.VideoCapture(0)
root = Tk()

labelZahl = Label(root, text='50')
labelZahl.pack()

def test():

    ret, frame = cap.read()

    # Our operations qon the frame come here
    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    cv2.imshow('output',grey)

    root.after(100, test())


root.mainloop()
    

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()