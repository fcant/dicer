import numpy as np
import cv2
import PIL
from PIL import Image
from PIL import ImageTk
from tkinter import *

root = Tk()

image = PIL.Image.open('bild.jpg')

photo = ImageTk.PhotoImage(image)
label = Label(root, image=photo)
label.pack()


# img = cv2.imread('bild.jpg',0)
# cv2.imshow('img', img)
# cv2.waitKey()

root.mainloop()
