import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False)

GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)
GPIO.setup(4, GPIO.OUT)

#steptime = 0.0002

while(True):
    time.sleep(3)
    GPIO.output(17, GPIO.LOW)

    

    for i in range(3200):
        if (i >= 2500 and i < 3100):
            steptime = 0.0006
        if (i>= 2900):
            steptime = 0.0009
        else:
            steptime = 0.0002
        GPIO.output(4, GPIO.HIGH)
        time.sleep(steptime)
        GPIO.output(4, GPIO.LOW)
        time.sleep(steptime)   
    

#    time.sleep(5)
#    GPIO.output(17, GPIO.HIGH)
        
#    for y in range(100):
#        GPIO.output(4, GPIO.HIGH)
#        time.sleep(0.01)
#        GPIO.output(4, GPIO.LOW)
#       time.sleep(0.01)

