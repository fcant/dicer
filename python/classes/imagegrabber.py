import numpy as np
import cv2
from tkinter import *
from PIL import ImageTk,Image
cap = cv2.VideoCapture(0)

def get_image():
    
    for i in range(2):
        ret, frame = cap.read()


    y=200
    h=230
    
    x=260
    w=250

    frame = frame[y:y + h, x:x + w]
    
    frame = cv2.flip(frame, 1)
    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    cv2image = cv2.cvtColor(grey, cv2.COLOR_BGR2RGBA)
    
    img = Image.fromarray(cv2image)
    imgtk = ImageTk.PhotoImage(image=img)
    raw_image.imgtk = imgtk
    raw_image.configure(image=imgtk)

    return grey
    
