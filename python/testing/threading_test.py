import _thread
import time
import numpy as np
import cv2
import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False)

GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)
GPIO.setup(4, GPIO.OUT)

def step():
    while(True):
  #      GPIO.output(4, GPIO.HIGH)
 #       time.sleep(0.05)
 #       GPIO.output(4, GPIO.LOW)
#        time.sleep(0.05)
        time.sleep(1)
        print("Step")

# Define a function for the thread
def print_time( threadName, delay):
   count = 0
   while(True):
 #     time.sleep(delay)
 #     count += 1
  #    print ("%s: %s" % ( threadName, time.ctime(time.time()) ))
      time.sleep(1)
      print("time")

# Create two threads as follows
try:
   _thread.start_new_thread(step())
except:
   print ("Error: unable to start thread")

try:
   _thread.start_new_thread( print_time, ("Thread-2", 4, ) )
except:
   print ("Error: unable to start thread")


while 1:
   pass