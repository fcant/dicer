import threading
import numpy as np
import cv2
from tkinter import *
from PIL import ImageTk,Image
import RPi.GPIO as GPIO
import time
#import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

root = Tk()
root.title('Dicer')

GPIO.setwarnings(False)

GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)
GPIO.setup(4, GPIO.OUT)

blob_params = cv2.SimpleBlobDetector_Params()

blob_params.filterByColor = True
blob_params.filterByArea = True
blob_params.minArea = 100
blob_params.filterByCircularity = False
blob_params.filterByInertia = True
blob_params.filterByConvexity = True

rollnumber=0
one=0
two=0
three=0
four=0
five=0
six=0

ready  = 0
errorcnt = 0

steptime = 0.0003

#ret, frame = cap.read() #ret gibt true oder false zurück, checkt ob video läuft

#def forward():
#    stand = int(labelZahl.cget('text'))
#    stand = stand + 1
#    labelZahl.config(text=str(stand))
    
    
stepper_running = 0
imgshow_running = 0
ready_for_img = 0

class stepperThread (threading.Thread):
    def __init__(self, threadID, name, counter):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.counter = counter
    def run(self):
        global stepper_running
        global ready_for_img
        ready_for_img = 0
        stepper_running = 1
        stepper()
        ready_for_img = 1
        print ("============Exiting " + self.name)
      
class showInputThread (threading.Thread):
    def __init__(self, threadID, name, counter):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.counter = counter
    def run(self):
        global imgshow_running
        imgshow_running = 1
        grey = get_image()
        show_input(grey)
        keypoint_img = img_processing(grey)
        number = counting(keypoint_img)
        show_output(keypoint_img)
        imgshow_running = 0
      

def stepper():

    time.sleep(0.5)
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


def slider_plus():
    stand = binary_slider.get()
    stand = stand + 1
    binary_slider.set(stand)
    
def slider_minus():
    stand = binary_slider.get()
    stand = stand - 1
    binary_slider.set(stand)

def step_plus():
    GPIO.output(17, GPIO.LOW)
    GPIO.output(4, GPIO.HIGH)
    time.sleep(steptime)
    GPIO.output(4, GPIO.LOW)
    time.sleep(steptime)
    
def step_minus():
    GPIO.output(17, GPIO.HIGH)
    GPIO.output(4, GPIO.HIGH)
    time.sleep(steptime)
    GPIO.output(4, GPIO.LOW)
    time.sleep(steptime)
    GPIO.output(17, GPIO.LOW)    

def get_image():
    
    for i in range(2):
        ret, frame = cap.read()
   
    y=200
    h=230
    
    x=280
    w=220

    frame1 = frame[y:y + h, x:x + w]  
    grey = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
    return grey


def img_processing(grey):
    
    ret, binary_image = cv2.threshold(grey, 230, 255, cv2.THRESH_BINARY)

    kernel = np.ones((6, 6), np.uint8)

    opening = cv2.morphologyEx(binary_image, cv2.MORPH_OPEN, kernel)
    erosion = cv2.erode(opening, kernel, iterations = 1)
    
    #closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)

    closing =  erosion
    closing = cv2.bitwise_not(closing)

    w = closing.shape[1]  # y
    h = closing.shape[0]  # x

    mask = np.zeros((h + 2, w + 2), np.uint8)
    cv2.floodFill(closing, mask, (0, 0), 255);
    cv2.floodFill(closing, mask, (0, 200), 255);
    
    return closing

def counting(image):
    
    global rollnumber
    global one
    global two
    global three
    global four
    global five
    global six
    global errorcnt
    
    detector = cv2.SimpleBlobDetector_create(blob_params)
    keypoints = detector.detect(image)
    img_with_keypoints = cv2.drawKeypoints(image, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    
    number = 0
    
    for i in keypoints[0:]:
        number = number + 1
    #print(number)
    
    print('counting')
    
    if number == 1:
        one = one +1
        detected_number.config(text='1')
    elif number == 2:    
        two = two +1
        detected_number.config(text='2')
    elif number == 3:    
        three = three +1
        detected_number.config(text='3')
    elif number == 4:    
        four = four +1
        detected_number.config(text='4')
    elif number == 5:
        five = five +1
        detected_number.config(text='5')
    elif number == 6:
        six = six + 1
        detected_number.config(text='6')
 #   elif number > 6 or number < 1:
  #      cv2.imwrite(str(errorcnt) + 'error.png', grey)
   #     errorcnt = errorcnt + 1
    #    print('FEHLER')   
     #   if errorcnt == 100:
      #      errorcnt = 1
      
    rollnumber = rollnumber + 1  
    
    all_numbers = [one, two, three, four, five, six, rollnumber, errorcnt]
    return all_numbers
    
def logging(number):
    
    print('logging')
    
    file = open('log', 'w')
    file.write('Einz:' + str(number[0]) + '\n')
    file.write('Zwei:' + str(number[1]) + '\n')    
    file.write("Drei: " + str(number[2]) + '\n')
    file.write("Vier: " + str(number[3]) + '\n')
    file.write("Fuenf: " + str(number[4]) + '\n')
    file.write("Sechs: " + str(number[5]) + '\n')
    file.write('Gesamt: ' + str(number[6]) + '\n')
    file.write('Fehler: ' + str(number[7]) + '\n')
    file.close()   
  
    values = [number[0],number[1],number[2],number[3],number[4],number[5]]
    
    ax.cla()
    ax.bar([1,2,3,4,5,6], values)
    canvas.draw()
    print('processing finish')    
    
    
def show_input(grey):
   
    #if bin_true.get() == 1:
    #    ret, grey = cv2.threshold(grey, int(binary_slider.get()) , 255, cv2.THRESH_BINARY)
        
    cv2image1 = cv2.cvtColor(grey, cv2.COLOR_BGR2RGBA)
    img1 = Image.fromarray(cv2image1)
    imgtk1 = ImageTk.PhotoImage(image=img1)
    raw_image.imgtk = imgtk1
    raw_image.configure(image=imgtk1)
 
def show_output(img_with_keypoints):
    
    cv2image2 = cv2.cvtColor(img_with_keypoints, cv2.COLOR_BGR2RGBA)
    img2 = Image.fromarray(cv2image2)
    imgtk2 = ImageTk.PhotoImage(image=img2)
    output_image.imgtk = imgtk2
    output_image.configure(image=imgtk2)

def mainprogram():
    global stepper_running
    
    #print('stepper_running: ' + str(stepper_running))
    #print('readyfi: ' + str(ready_for_img))
    if start_stop.get() == 1:
        if stepper_running == 0:
            thread1 = stepperThread(1, "Thread-1", 1)
            thread1.start()
        if ready_for_img == 1:
            
            grey = get_image()
            show_input(grey)
            keypoint_img = img_processing(grey)
            number = counting(keypoint_img)
            logging(number)
            show_output(keypoint_img)
            stepper_running = 0
            print('image finished')
        
        
    elif start_stop.get() == 0 and imgshow_running == 0:
        thread2 = showInputThread(1, "Thread-1", 1)
        thread2.start()
    
    topFrame.after(1000, mainprogram)  


topFrame = Frame(root)
topFrame.pack(side=TOP)

bottomFrame = Frame(root)
bottomFrame.pack(side=BOTTOM)

bin_true=IntVar()
start_stop=IntVar()

Checkbutton(bottomFrame, text="Binary", variable=bin_true).grid(row=1, column=0)
Checkbutton(bottomFrame, text="würfeln", variable=start_stop).grid(row=1, column=5)

Button(bottomFrame, text='-', command=slider_minus).grid(row=1, column=1)

binary_slider = Scale(bottomFrame, from_=0, to=255, orient=HORIZONTAL)
binary_slider.grid(row=1, column=2)

Button(bottomFrame, text='+', command=slider_plus).grid(row=1, column=3)

Button(bottomFrame, text='Step up', command=step_plus).grid(row=2, column=1)
Button(bottomFrame, text='Step down', command=step_minus).grid(row=2, column=2)

cap = cv2.VideoCapture(0)

dummy_image = PhotoImage(file='dummy_image.png')

raw_image = Label(topFrame, image=dummy_image)
raw_image.grid(row=0, column=0)

output_image = Label(topFrame, image=dummy_image)
output_image.grid(row=0, column=1)

detected_number = Label(topFrame, text='0', padx=50, pady=50)
detected_number.config(font=("Courier", 44))
detected_number.grid(row=0, column=2)

fig = Figure()   
ax = fig.add_subplot(111)
ax.set_xlabel('Augenzahlen')
ax.set_ylabel('Häufigkeit')
ax.grid
        
canvas = FigureCanvasTkAgg(fig, topFrame)
canvas.get_tk_widget().grid(row=0, column=4)
canvas.draw()

#image = get_image()
#show_input(image)
#processed_image =  counting(image)
#show_output(processed_image)




mainprogram()
root.mainloop()


#img = ImageTk.PhotoImage()
#panel = Label(root, image = img)
#panel.pack()

#
