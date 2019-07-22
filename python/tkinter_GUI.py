import numpy as np
import cv2
from tkinter import *
from PIL import ImageTk,Image
import RPi.GPIO as GPIO
import time


#cap = cv2.VideoCapture(0)

root = Tk()

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

wurfzahl=0
one=0
two=0
three=0
four=0
five=0
six=0


errorcnt = 0

steptime = 0.0003

#ret, frame = cap.read() #ret gibt true oder false zurück, checkt ob video läuft



#def forward():
#    stand = int(labelZahl.cget('text'))
#    stand = stand + 1
#    labelZahl.config(text=str(stand))
    
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

Button(bottomFrame, text='Step +', command=step_plus).grid(row=2, column=1)
Button(bottomFrame, text='Step -', command=step_minus).grid(row=2, column=2)

cap = cv2.VideoCapture(0)

dummy_image = PhotoImage(file='dummy_image.png')

raw_image = Label(topFrame, image=dummy_image)
raw_image.grid(row=0, column=0)

output_image = Label(topFrame, image=dummy_image)
output_image.grid(row=0, column=1)

detected_number = Label(topFrame, text='0', padx=50, pady=50)
detected_number.config(font=("Courier", 44))
detected_number.grid(row=0, column=2)

def mainprogram():
    image = get_image()
    #show_frame(image)
    counting(image)
    if start_stop.get() == 1:
        print(start_stop.get())
        stepper()
    topFrame.after(10, mainprogram)

def stepper():
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


def get_image():
    
    for i in range(2):
        ret, frame = cap.read()

    
    y=200
    h=230
    
    x=260
    w=250

    frame = frame[y:y + h, x:x + w]
    
    frame = cv2.flip(frame, 1)
    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    cv2image = cv2.cvtColor(grey, cv2.COLOR_BGR2RGBA)
    
    img = Image.fromarray(cv2image)
    imgtk = ImageTk.PhotoImage(image=img)
    raw_image.imgtk = imgtk
    raw_image.configure(image=imgtk)
    
    return grey


def counting(grey):
    
    global wurfzahl
    global one
    global two
    global three
    global four
    global five
    global six
    global errorcnt
    
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

    detector = cv2.SimpleBlobDetector_create(blob_params)
    keypoints = detector.detect(closing)

    img_with_keypoints = cv2.drawKeypoints(closing, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    
    
    
    number = 0
    
    for i in keypoints[0:]:
        number = number + 1
    #print(number)
    

    
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
    
    file = open('log', 'w')
    file.write('Einz:' + str(one) + '\n')
    file.write('Zwei:' + str(two) + '\n')    
    file.write("Drei: " + str(three) + '\n')
    file.write("Vier: " + str(four) + '\n')
    file.write("Fuenf: " + str(five) + '\n')
    file.write("Sechs: " + str(six) + '\n')
    file.write('Gesamt: ' + str(wurfzahl) + '\n')
    file.write('Fehler: ' + str(errorcnt) + '\n')
    file.close()
    
    
    wurfzahl = wurfzahl + 1     

    print("=======================")
    print("Einz: ", one)
    print("Zwei: ", two)
    print("Drei: ", three)
    print("Vier: ", four)
    print("Fuenf: ", five)
    print("Sechs: ", six)
    print('Gesamt: ', wurfzahl)

    cv2image = cv2.cvtColor(img_with_keypoints, cv2.COLOR_BGR2RGBA)
    
    img = Image.fromarray(cv2image)
    imgtk = ImageTk.PhotoImage(image=img)
    output_image.imgtk = imgtk
    output_image.configure(image=imgtk)



def show_frame(grey):
   
    if bin_true.get() == 1:
        ret, grey = cv2.threshold(grey, int(binary_slider.get()) , 255, cv2.THRESH_BINARY)
        
    cv2image = cv2.cvtColor(grey, cv2.COLOR_BGR2RGBA)
    
    img = Image.fromarray(cv2image)
    imgtk = ImageTk.PhotoImage(image=img)
    raw_image.imgtk = imgtk
    raw_image.configure(image=imgtk)
    

mainprogram()
root.mainloop()


#img = ImageTk.PhotoImage()
#panel = Label(root, image = img)
#panel.pack()

#