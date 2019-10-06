import time
import threading
from tkinter import *


class ClockThread(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self)

    def run(self):
        now = time.time()
        while clock_run == True:
            clock(now)




root = Tk()


clock_run = False

sec = 0
min = 0
hr = 0

def clock(now):

    # showTime = time.strftime("%S:%H:%M:", time.ctime(time.time()-now))


    time_seconds = int((time.time()-now))


    hr = int(time_seconds/3600)
    min = int(time_seconds/60)-(hr*60)
    sec = int(time_seconds)-(min*60)

    showTime = str(hr).zfill(3),':', str(min).zfill(2),':', str(sec).zfill(2)

    timer.config(text=showTime)

#(now.day, now.month, now.year, now.hour, now.minute, now.second))

def start_clock():
    global clock_run
    if clock_run == True:
        clock_run = False
        clock_control.config(text='Start')
    else:
        clock_run = True
        clock_control.config(text='Stop')
        thread1 = ClockThread()
        thread1.start()


clock_control = Button(root, text='Start', command=start_clock)
clock_control.pack()

timer = Label(root, text='00:00')
timer.pack()

mainloop()


