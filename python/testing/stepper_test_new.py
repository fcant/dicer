import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False)

GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)
GPIO.setup(4, GPIO.OUT)

step = 10

for u in range(20000):
    GPIO.output(17, GPIO.LOW)
    if(u == 10000):
        print("asd")
        GPIO.output(4, GPIO.HIGH)
    else:
        GPIO.output(4, GPIO.LOW)
 

  #  for i in range(100):
   #     GPIO.output(4, GPIO.HIGH)
    #    time.sleep(0.01)
     #   GPIO.output(4, GPIO.LOW)
      #  time.sleep(0.01)