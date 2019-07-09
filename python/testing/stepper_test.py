import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False)

GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)
GPIO.setup(4, GPIO.OUT)



GPIO.output(4, GPIO.HIGH)
time.sleep(0.01)
GPIO.output(4, GPIO.LOW)
time.sleep(0.01)

#for u in range(20):





#    time.sleep(1)
 #   GPIO.output(17, GPIO.LOW)
#
 #   for i in range(100):
  #      GPIO.output(4, GPIO.HIGH)
   #     time.sleep(0.01)
    #    GPIO.output(4, GPIO.LOW)
     #   time.sleep(0.01)

    #time.sleep(1)
#    GPIO.output(17, GPIO.HIGH)
 #       
  #  for y in range(100):
   #     GPIO.output(4, GPIO.HIGH)
    #    time.sleep(0.01)
     #   GPIO.output(4, GPIO.LOW)
      #  time.sleep(0.01)

