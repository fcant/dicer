
import threading
import numpy as np
import cv2
from tkinter import *
from PIL import ImageTk,Image
import time
#import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

root = Tk()

topFrame = Frame(root)
topFrame.pack(side=TOP)

fig1 = Figure()
fig1.subplots_adjust(bottom=0.3)
fig1.set_size_inches(5, 1.5)

ax = fig1.add_subplot(111)

ax.barh([0],(50))
ax.barh([1],(10))
ax.set_yticks((0,1))
ax.set_yticklabels(('Würfe', 'Fehler'))
ax.set_xlabel('Anzahl')
ax.invert_yaxis()

canvas1 = FigureCanvasTkAgg(fig1, topFrame)
canvas1.get_tk_widget().grid(row=0, column=4)
canvas1.draw()


root.mainloop()