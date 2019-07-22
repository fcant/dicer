import numpy as np
import cv2
from tkinter import *
from PIL import ImageTk,Image


#cap = cv2.VideoCapture(0)

root = Tk()

#ret, frame = cap.read() #ret gibt true oder false zurück, checkt ob video läuft



#def forward():
#    stand = int(labelZahl.cget('text'))
#    stand = stand + 1
#    labelZahl.config(text=str(stand))
    
def slider_plus():
    stand = binary_slider.get()
    stand = stand + 1
    binary_slider.set(stand)
    
def slider_minus():
    stand = binary_slider.get()
    stand = stand - 1
    binary_slider.set(stand)

topFrame = Frame(root)
topFrame.pack(side=TOP)

bottomFrame = Frame(root)
bottomFrame.pack(side=BOTTOM)

bin_true=IntVar()

#check_binary = Checkbutton(root, text="Binary", variable=neg)
#check_binary.grid(row=0)


#labelZahl = Label(bottomFrame, text='0')
#labelZahl.pack()

#button1 = Button(bottomFrame, text='+', fg='red', command=forward)
#button1.pack()

#button2 = Button(bottomFrame, text='-', fg='blue', command=back)
#button2.pack()

Checkbutton(bottomFrame, text="Binary", variable=bin_true).grid(row=1, column=0)

Button(bottomFrame, text='-', command=slider_minus).grid(row=1, column=1)

binary_slider = Scale(bottomFrame, from_=0, to=255, orient=HORIZONTAL)
binary_slider.grid(row=1, column=2)

Button(bottomFrame, text='+', command=slider_plus).grid(row=1, column=3)

cap = cv2.VideoCapture(0)

webcam_image = Label(topFrame)
webcam_image.grid(row=0,)

def mainprogram():
    image = get_image()
    show_frame(image)
    stepper()
    webcam_image.after(5, mainprogram)

def get_image():
    ret, frame = cap.read()
    
    y=200
    h=230
    
    x=260
    w=250

    frame = frame[y:y + h, x:x + w]
    
    frame = cv2.flip(frame, 1)
    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return grey


def show_frame(grey):
   
    if bin_true.get() == 1:
        ret, grey = cv2.threshold(grey, int(binary_slider.get()) , 255, cv2.THRESH_BINARY)
        
    cv2image = cv2.cvtColor(grey, cv2.COLOR_BGR2RGBA)
    
    
    img = Image.fromarray(cv2image)
    imgtk = ImageTk.PhotoImage(image=img)
    webcam_image.imgtk = imgtk
    webcam_image.configure(image=imgtk)





mainprogram()    
root.mainloop()


#img = ImageTk.PhotoImage()
#panel = Label(root, image = img)
#panel.pack()

#
