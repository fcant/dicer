import numpy as np
import cv2
from tkinter import *
from PIL import ImageTk,Image


#cap = cv2.VideoCapture(0)

root = Tk()

#ret, frame = cap.read() #ret gibt true oder false zurück, checkt ob video läuft



def forward():
    stand = int(labelZahl.cget('text'))
    stand = stand + 1
    labelZahl.config(text=str(stand))


def back():
    stand = int(labelZahl.cget('text'))
    stand = stand - 1
    labelZahl.config(text=str(stand))




labelZahl = Label(root, text='50')
labelZahl.pack()

topFrame = Frame(root)
topFrame.pack(side=TOP)

bottomFrame = Frame(root)
bottomFrame.pack(side=BOTTOM)

neg=IntVar()
C1 = Checkbutton(bottomFrame, text="negativ", variable=neg).grid(row=0, sticky=W)

button1 = Button(bottomFrame, text='+', fg='red', command=forward)
button2 = Button(bottomFrame, text='-', fg='blue', command=back)

cap = cv2.VideoCapture(0)

brightness = int(labelZahl.cget('text'))

#button1.pack(side=TOP)
#button2.pack(side=LEFT)

lmain = Label(root)
lmain.pack()


def show_frame():
    cv2.VideoCapture.set(cap, 10, brightness)
    ret, frame = cap.read()
    frame = cv2.flip(frame, 1)
    cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)
    img = Image.fromarray(cv2image)
    imgtk = ImageTk.PhotoImage(image=img)
    lmain.imgtk = imgtk
    lmain.configure(image=imgtk)
    lmain.after(5, show_frame)


show_frame()
root.mainloop()


#img = ImageTk.PhotoImage()
#panel = Label(root, image = img)
#panel.pack()

#
