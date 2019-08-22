import threading
import datetime
import numpy as np
import cv2
from tkinter import *
from PIL import ImageTk,Image
import RPi.GPIO as GPIO
import time
#import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

starting = 0

root = Tk()
root.title('Dicer V0.6')

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
    
taking_image = 0
stepper_running = 0
imgshow_running = 0

class stepperThread (threading.Thread):

    def __init__(self, threadID, name, counter):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.counter = counter
    def run(self):
        global stepper_running
        global taking_image
        #print ("============Starting " + self.name)
        stepper()
        taking_image = 1
        stepper_running = 0
      
        #print ("============Exiting " + self.name)

class liveView (threading.Thread):
    global imgshow_running
    def __init__(self, threadID, name, counter):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.counter = counter
    def run(self):
        imgshow_running = 1
        #print ("============Starting " + self.name)
        grey = get_image()
        show_raw()
        keypoint_img = img_processing(grey)
        number = counting(keypoint_img)
        show_output()
        #print ("============Exiting " + self.name)
        imgshow_running = 0
      

def stepper():
    for i in range(3200):        
        if (i > 2931):
            steptime = np.sin((i-2900)*0.00001)
        else:
            steptime = 0.0003
        GPIO.output(4, GPIO.HIGH)
        time.sleep(steptime)
        GPIO.output(4, GPIO.LOW)
        time.sleep(steptime)
    time.sleep(1)

def reset():
    global rollnumber
    global one
    global two
    global three
    global four
    global five
    global six
    global errorcnt
    
    rollnumber=0
    one=0
    two=0
    three=0
    four=0
    five=0
    six=0
    errorcnt=0
    
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
    
    for i in range(5):
        ret, frame = cap.read()
   
    y=200
    h=230
    
    x=280
    w=220

    frame1 = frame[y:y + h, x:x + w]  
    grey = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
    cv2.imwrite('last_raw.png', grey)    
    return grey


def img_processing(image_input):
    
    print(dark_numbers.get())
    
    #input_frame = cv2.cvtColor(image_input, cv2.COLOR_BGR2GRAY) #Kamerabild in Graustufen umwandeln

    ret, binary_image = cv2.threshold(image_input, binary_slider.get(), 255, cv2.THRESH_BINARY) #Schwellenwertbild abspeichern
    
    kernel_rect = np.ones((5, 5), np.uint8) #quadratische Maske erzeugen
    opening = cv2.morphologyEx(binary_image, cv2.MORPH_OPEN, kernel_rect)
    
    if dark_numbers.get() == 1:
       
        w = opening.shape[1]  # y
        h = opening.shape[0]  # x

        mask = np.zeros((h + 2, w + 2), np.uint8)
    
        cv2.floodFill(opening, mask, (0, 0), 255);
        cv2.floodFill(opening, mask, (0, 200), 255)
        
    else:   
        opening = cv2.bitwise_not(opening)


    kernel_round = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(12,12)) #Ellipse als Maske erzeugen, un Punktförmigkeit der Augenzahlen beizubehalten
    dilate = cv2.dilate(opening, kernel_round, iterations = 1) #zweimal Erosion anwenden

    kernel_round = np.array([[0,0,0,1,1,1,0,0,0],
                             [0,1,1,1,1,1,1,1,0],
                             [0,1,1,1,1,1,1,1,0],
                             [1,1,1,1,1,1,1,1,1],
                             [1,1,1,1,1,1,1,1,1],
                             [1,1,1,1,1,1,1,1,1],
                             [0,1,1,1,1,1,1,1,0],
                             [0,1,1,1,1,1,1,1,0],
                             [0,0,0,1,1,1,0,0,0]], dtype=np.uint8) #Kreisförmige Maske erzeugen
    
    erode= cv2.erode(dilate, kernel_round, iterations = 1)   #Dilatation anwenden, um weiße Punkte wieder zu vergrößern 

    
    #cv2image2 = cv2.cvtColor(closing, cv2.COLOR_BGR2RGBA)
    #img2 = Image.fromarray(cv2image2)
    #imgtk2 = ImageTk.PhotoImage(image=img2)
    #output_image.imgtk = imgtk2
    #output_image.configure(image=imgtk2)   
    #print('processing finish')   
    
    return erode

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
    
    cv2.imwrite('last_output.png', img_with_keypoints)
    
    number = 0
    
    for i in keypoints[0:]:
        number = number + 1
    if start_stop.get() == 1:    
        rollnumber = rollnumber + 1

    detected_number.config(text=str(number))

    if start_stop.get() == 1 and taking_image == 1 and number == 1:
        one = one +1
    elif start_stop.get() == 1 and taking_image == 1 and number ==2:
        two = two +1
    elif start_stop.get() == 1 and taking_image == 1 and number ==3:
        three = three +1
    elif start_stop.get() == 1 and taking_image == 1 and number == 4:        
        four = four +1
    elif start_stop.get() == 1 and taking_image == 1 and number ==5:        
        five = five +1
    elif start_stop.get() == 1 and taking_image == 1 and number == 6:        
        six = six + 1
    elif (number > 6 or number < 1) and taking_image == 1:
        if start_stop.get() == 1:
            errorcnt = errorcnt + 1
            if error_logging == 1:
                cv2.imwrite('errors/'+ str(errorcnt) + 'error PROCESSED.png', image)
                raw = cv2.imread('last_raw.png')           
                cv2.imwrite('errors/' + str(errorcnt) + ' error RAW.png', raw)            
            
         
    all_numbers = [one, two, three, four, five, six, errorcnt,rollnumber]
    return all_numbers
    
def logging(numbers):
    file = open('log', 'w')
    file.write('Einz:' + str(numbers[0]) + '\n')
    file.write('Zwei:' + str(numbers[1]) + '\n')    
    file.write("Drei: " + str(numbers[2]) + '\n')
    file.write("Vier: " + str(numbers[3]) + '\n')
    file.write("Fuenf: " + str(numbers[4]) + '\n')
    file.write("Sechs: " + str(numbers[5]) + '\n')
    file.write('Fehler: ' + str(numbers[6]) + '\n')
    file.write('Gesamt: ' + str(numbers[7]) + '\n')
    file.close()   
  
  
    all_rolls.config(text=str(numbers[7]))
    errors.config(text=str(numbers[6]))
 
    values = [numbers[0],numbers[1],numbers[2],numbers[3],numbers[4],numbers[5]]
    
    ax.cla()
    ax.set_xlabel('Augenzahlen')
    ax.set_ylabel('Häufigkeit')
    ax.bar([1,2,3,4,5,6], values)
 
    canvas1.draw()

def show_raw():
    if bin_true.get() == 1:
        raw = cv2.imread('last_raw.png')
        ret, binary_image = cv2.threshold(raw, int(binary_slider.get()) , 255, cv2.THRESH_BINARY)
        cv2.imwrite('last_bin.png', binary_image)
        img2 = ImageTk.PhotoImage(Image.open('last_bin.png'))   
        raw_image.configure(image=img2)
        raw_image.image = img2 
    else:
        img2 = ImageTk.PhotoImage(Image.open('last_raw.png'))   
        raw_image.configure(image=img2)
        raw_image.image = img2   
        
    #cv2image1 = cv2.cvtColor(grey, cv2.COLOR_BGR2RGBA)
    #img1 = Image.fromarray(cv2image1)
    #imgtk1 = ImageTk.PhotoImage(image=img1)
    #raw_image.imgtk = imgtk1
    #raw_image.configure(image=imgtk1)
 
def show_output():
    
    img2 = ImageTk.PhotoImage(Image.open('last_output.png'))
    output_image.configure(image=img2)
    output_image.image = img2
    
    #cv2image2 = cv2.cvtColor(img_with_keypoints, cv2.COLOR_BGR2RGBA)
    #img2 = Image.fromarray(cv2image2)
    #imgtk2 = ImageTk.PhotoImage(image=img2)
    #output_image.imgtk = imgtk2
    #output_image.configure(image=imgtk2)

def save_image():
    raw = cv2.imread('last_raw.png')
    ts = datetime.datetime.now().timestamp()
    cv2.imwrite(str(ts) + '.png', raw) 

def mainprogram():
    global stepper_running
    global taking_image
    
    file = open('config', 'w')
    file.write(str(binary_slider.get()))
    file.close() 

    if start_stop.get() == 1 and imgshow_running == 0:
        if stepper_running == 0 and taking_image == 0:
            stepper_running = 1
            thread1 = stepperThread(1, "stepper Thread", 1)
            thread1.start()
        if stepper_running == 0 and taking_image == 1:
            raw_image = get_image()
            show_raw()
            processed_img = img_processing(raw_image)
            numbers = counting(processed_img)
            taking_image = 0
            show_output()
            logging(numbers)
        root.after(100, mainprogram)  

        
        
    elif start_stop.get() == 0:
        if imgshow_running == 0: #live View
            thread2 = liveView(2, "liveView Thread", 2)
            thread2.start()
        root.after(1000, mainprogram)
        
    root.update()

cap = cv2.VideoCapture(0)

topFrame = Frame(root)
topFrame.pack(side=TOP)

bottomFrame = Frame(root)
bottomFrame.pack(side=LEFT)

error_logging = IntVar()
bin_true=IntVar()
start_stop=IntVar()
dark_numbers=IntVar()

Checkbutton(bottomFrame, text="binary", variable=bin_true).grid(row=1, column=4)
Checkbutton(bottomFrame, text="roll", variable=start_stop).grid(row=1, column=5)
Checkbutton(bottomFrame, text="Error logging", variable=error_logging).grid(row=1, column=6)
Checkbutton(bottomFrame, text="dark numbers", variable=dark_numbers).grid(row=3, column=4)

Label(bottomFrame, text='Binary value: ').grid(row=3, column=0, sticky=E, padx=10)

Button(bottomFrame, text='-', command=slider_minus).grid(row=3, column=1, sticky=E)

binary_slider = Scale(bottomFrame, from_=0, to=255, orient=HORIZONTAL)
binary_slider.grid(row=3, column=2)

file = open('config', 'r')
old_bin = file.read()
binary_slider.set(int(old_bin))
file.close()

Button(bottomFrame, text='+', command=slider_plus).grid(row=3, column=3, sticky=W)

Button(bottomFrame, text='Save Image', command=save_image).grid(row=1, column=8, sticky=E)
Button(bottomFrame, text='Step up', command=step_plus).grid(row=1, column=10)
Button(bottomFrame, text='Step down', command=step_minus).grid(row=1, column=11)
Button(bottomFrame, text='Reset', command=reset).grid(row=0, column=0, rowspan=2,padx=5, pady=5, sticky=N)

dummy_image = PhotoImage(file='dummy_image.png')

Label(bottomFrame, text='Motor Control: ').grid(row=1, column=9, sticky=E, padx=20)
raw_image = Label(topFrame, image=dummy_image)
raw_image.grid(row=0, column=0)

output_image = Label(topFrame, image=dummy_image)
output_image.grid(row=1, column=0)

Label(bottomFrame, text='All rolls: ').grid(row=0, column=1, sticky=W)
all_rolls = Label(bottomFrame, text='0')
all_rolls.grid(row=0, column=2, sticky=W)

Label(bottomFrame, text='Errors: ').grid(row=1, column=1, sticky=W)
errors = Label(bottomFrame, text='0')
errors.grid(row=1, column=2, sticky=W)

detected_number = Label(topFrame, text='0', padx=50, pady=50)
detected_number.config(font=("Courier", 44))
detected_number.grid(row=0, column=2, rowspan=2)

fig1 = Figure()   
ax = fig1.add_subplot(111)
ax.set_xlabel('Numbers')
ax.set_ylabel('Count')

canvas1 = FigureCanvasTkAgg(fig1, topFrame)
canvas1.get_tk_widget().grid(row=0, column=4, rowspan=2)
canvas1.draw()

mainprogram()
root.mainloop()