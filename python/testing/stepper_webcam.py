import RPi.GPIO as GPIO
import time
#import numpy as np
import cv2



GPIO.setwarnings(False)

GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)
GPIO.setup(4, GPIO.OUT)

cap = cv2.VideoCapture(0)

steptime = 0.0003
while(True):
    
    for i in range(3200):
        GPIO.output(4, GPIO.HIGH)
        time.sleep(steptime)
        GPIO.output(4, GPIO.LOW)
        time.sleep(steptime)  

#while(True):
    # Capture frame-by-frame

    time.sleep(2)

    for i in range(10):
        ret, frame = cap.read()

    # Our operations qon the frame come here
    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    cv2.imwrite("stepper_test.png", grey)
    # Display the resulting frame
    output = cv2.imread('stepper_test.png',1)

    cv2.imshow('output',output)
   
    if cv2.waitKey() & 0xFF == ord('q'):
        break
    

    
    
        
cap.release()
cv2.destroyAllWindows()



# When everything done, release the capture