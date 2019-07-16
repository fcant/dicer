import RPi.GPIO as GPIO
import time
import numpy as np
import cv2



GPIO.setwarnings(False)

GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)
GPIO.setup(4, GPIO.OUT)

cap = cv2.VideoCapture(0)

# Capture frame-by-frame
ret, frame = cap.read()

# Our operations on the frame come here
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

# Display the resulting frame
cv2.imshow('frame',gray)

z=0
steptime = 0.0002

while(z==1):

    for i in range(3200):
     #   if (i >= 2500 and i < 3100):
      #      steptime = 0.0006
      #  if (i>= 2900):
       #     steptime = 0.0009
      #  else:
      #      steptime = 0.0002
        GPIO.output(4, GPIO.HIGH)
        time.sleep(steptime)
        GPIO.output(4, GPIO.LOW)
        time.sleep(steptime)   
      
#    if cv2.waitKey(1) & 0xFF == ord('q'):
#        break
#    
#        # Capture frame-by-frame
#    ret, frame = cap.read()#

    # Our operations on the frame come here
   # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Display the resulting frame
    #cv2.imshow('frame',gray)
    
cap.release()
cv2.destroyAllWindows()
    
    
    
    
    

  #  for i in range(100):
   #     GPIO.output(4, GPIO.HIGH)
    #    time.sleep(0.01)
     #   GPIO.output(4, GPIO.LOW)
      #  time.sleep(0.01)