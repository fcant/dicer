import RPi.GPIO as GPIO
import time
import threading

exitFlag = 0
GPIO.setwarnings(False)

GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)
GPIO.setup(4, GPIO.OUT)

running = 0

class stepperThreading (threading.Thread):
   def __init__(self, threadID, name, counter):
      threading.Thread.__init__(self)
      self.threadID = threadID
      self.name = name
      self.counter = counter
   def run(self):
      print ("Starting " + self.name)
      stepper()
      print ("Exiting " + self.name)

def stepper():
    global running
    running = 1
    for i in range(3200):
        if (i > 3000):
            steptime = 0.0006
        else:
            steptime = 0.0003
        GPIO.output(4, GPIO.HIGH)
        time.sleep(steptime)
        GPIO.output(4, GPIO.LOW)
        time.sleep(steptime)
    time.sleep(2)
    running = 0


# Create new threads
#thread1 = myThread(1, "Thread-1", 1)
#thread2 = myThread(2, "Thread-2", 2)

# Start new Threads
#thread1.start()
#thread2.start()

#print ("Exiting Main Thread")


while(True):

    
    
    ret, frame = cap.read()
    # Our operations qon the frame come here
    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.imshow('output',grey)
    if cv2.waitKey(1) & 0xFF == ord('s'):
        if running == 0:
            thread1 = myThread(1, "Thread-1", 1)
            thread1.start()
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()



