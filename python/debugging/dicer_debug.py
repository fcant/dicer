import threading
import datetime
import time
import numpy as np
import cv2
from tkinter import *
from PIL import ImageTk, Image
import RPi.GPIO as GPIO
import time
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import smtplib
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart

darknumbers=False

write_email = False  # Email mit Messdaten versenden?
email_log_number = 500  # Nach wie vielen Würfen soll eine Email geschrieben werden

# Emailserver konfigurieren und starten


cap = cv2.VideoCapture(0)

global_steptime = 0.00015

# Fleckendetektor konfigurieren
blob_params = cv2.SimpleBlobDetector_Params()
blob_params.filterByColor = True
blob_params.filterByArea = True
blob_params.minArea = 100
blob_params.filterByCircularity = False
blob_params.filterByInertia = True
blob_params.filterByConvexity = True

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)
GPIO.setup(4, GPIO.OUT)


rollnumber = 0
one = 0
two = 0
three = 0
four = 0
five = 0
six = 0
std= 0
errorcnt = 0
last_number = 0

longest = [0] * 6
row = [0] * 6

def step_plus():
    GPIO.output(17, GPIO.LOW)
    GPIO.output(4, GPIO.HIGH)
    time.sleep(global_steptime)
    GPIO.output(4, GPIO.LOW)
    time.sleep(global_steptime)
    GPIO.output(17, GPIO.LOW)
    print('step')


def step_minus():
    GPIO.output(17, GPIO.HIGH)
    GPIO.output(4, GPIO.HIGH)
    time.sleep(global_steptime)
    GPIO.output(4, GPIO.LOW)
    time.sleep(global_steptime)
    GPIO.output(17, GPIO.LOW)
    print('step')


def send_email(numbers):

    server = smtplib.SMTP('mail.gmx.net', 587)
    server.starttls()
    server.login('python-email@gmx.de', 'bojack123.')


    msg = MIMEMultipart()
    msg['From'] = 'python-email@gmx.de'
    msg['To'] = 'fabio.canterino@smail.th-koeln.de'
    msg['Subject'] = 'Dicer update'
    message = str(numbers[0]) + ',' + str(numbers[1]) + ',' + str(numbers[2]) + ',' + str(numbers[3]) + ',' + str(
        numbers[4]) + ',' + str(numbers[5]) + ' Err: ' + str(numbers[6]) + ' All: ' + str(numbers[6]) + ' Std: ' + str(
        numbers[7])
    msg.attach(MIMEText(message))

    server.send_message(msg)


def logging(numbers):

    longest_numbers = numbers[9]

    file = open('log', 'w')
    file.write('Einz:' + str(numbers[0]) + ';' + str(longest_numbers[0]) + '\n')
    file.write('Zwei:' + str(numbers[1]) + ';' + str(longest_numbers[1]) + '\n')
    file.write("Drei: " + str(numbers[2]) + ';' + str(longest_numbers[2]) + '\n')
    file.write("Vier: " + str(numbers[3]) + ';' + str(longest_numbers[3]) + '\n')
    file.write("Fuenf: " + str(numbers[4]) + ';' + str(longest_numbers[4]) + '\n')
    file.write("Sechs: " + str(numbers[5]) + ';' + str(longest_numbers[5]) + '\n')
    file.write('Fehler: ' + str(numbers[6]) + '\n')
    file.write('Gesamt: ' + str(numbers[7]) + '\n')
    file.write('Standardabw: ' + str(numbers[8]) + '\n')

    file.close()


def get_images():
    for i in range(5):
        ret, frame = cap.read()

    if not ret:
        grey = cv2.imread('dummy_image.png', 0)
        # grey = cv2.imread('INPUT.png', 0)
        cv2.putText(grey, 'NO CAMERA', (10, 200), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
    else:
        y = 110
        h = 300

        x = 200
        w = 300

        real_image = frame[y:y + h, x:x + w]
        grey = cv2.cvtColor(real_image, cv2.COLOR_BGR2GRAY)
        
        cv2.imwrite('Input_image.png',grey)
        cv2.imshow('Input_image',grey)

        y = 60
        h = 10

        x = 200
        w = 300
        
        pos_img = frame[y:y + h, x:x + w]
        pos_img = cv2.cvtColor(pos_img, cv2.COLOR_BGR2GRAY)
        ret, pos_img = cv2.threshold(pos_img, 240, 255,cv2.THRESH_BINARY)  # Schwellenwertbild abspeichern

    return grey, pos_img    


def img_processing(image_input):
    # input_frame = cv2.cvtColor(image_input, cv2.COLOR_BGR2GRAY) #Kamerabild in Graustufen umwandeln

    ret, binary_image = cv2.threshold(image_input, 220, 255,
                                      cv2.THRESH_BINARY)  # Schwellenwertbild abspeichern

    if darknumbers:

        w = binary_image.shape[1]  # y
        h = binary_image.shape[0]  # x

        mask = np.zeros((h + 2, w + 2), np.uint8)

        cv2.floodFill(binary_image, mask, (0, 0), 255);
        cv2.floodFill(binary_image, mask, (0, 200), 255)

        kernel_rect = np.ones((5, 5), np.uint8)  # quadratische Maske erzeugen

        clean_eyes = cv2.erode(binary_image, kernel_rect, iterations=1)  # zweimal Erosion anwenden

    else:
        binary_image = cv2.bitwise_not(binary_image)

        clean_eyes = binary_image

    cv2.imwrite('binary.png', binary_image)
    cv2.imshow('binary', binary_image) 

    kernel_round = np.array([[0, 0, 0, 1, 1, 1, 0, 0, 0],
                             [0, 1, 1, 1, 1, 1, 1, 1, 0],
                             [0, 1, 1, 1, 1, 1, 1, 1, 0],
                             [1, 1, 1, 1, 1, 1, 1, 1, 1],
                             [1, 1, 1, 1, 1, 1, 1, 1, 1],
                             [1, 1, 1, 1, 1, 1, 1, 1, 1],
                             [0, 1, 1, 1, 1, 1, 1, 1, 0],
                             [0, 1, 1, 1, 1, 1, 1, 1, 0],
                             [0, 0, 0, 1, 1, 1, 0, 0, 0]], dtype=np.uint8)  # Kreisförmige Maske erzeugen

    dilate = cv2.dilate(clean_eyes, kernel_round, iterations=4)  # Dilatation anwenden

    cv2.imwrite('Dilate.png', dilate)
    cv2.imshow('Dilate', dilate)

    erode = cv2.erode(dilate, kernel_round, iterations=2)  # Erosion anwenden
    
    cv2.imwrite('Erode.png', erode)
    cv2.imshow('Erode', erode)
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
    global last_number

    global longest
    global row

    detector = cv2.SimpleBlobDetector_create(blob_params)
    keypoints = detector.detect(image)
    img_with_keypoints = cv2.drawKeypoints(image, keypoints, np.array([]), (0, 0, 255),
                                           cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    number = 0

    for i in keypoints[0:]:
        number = number + 1

    rollnumber += 1
    print('DETECTED: ', number)
    cv2.putText(img_with_keypoints, str(number), (10, 250), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

    if number == 1:  
        one += 1
        if last_number == number:
            row[0] += 1
        else:
            row[0] = 1
    elif number == 2: 
        two += 1
        if last_number == number:
            row[1] += 1
        else:
            row[1] = 1
    elif number == 3: 
        three += 1
        if last_number == number:
            row[2] += 1
        else:
            row[2] = 1
    elif number == 4:
        four += 1
        if last_number == number:
            row[3] += 1
        else:
            row[3] = 1
    elif number == 5: 
        five += 1
        if last_number == number:
            row[4] += 1
        else:
            row[4] = 1
    elif number == 6:
        six += 1
        if last_number == number:
            row[5] += 1
        else:
            row[5] = 1
    else:
        errorcnt = errorcnt + 1
        cv2.imwrite('errors/' + str(errorcnt) + ' error.png', image)

    if row[0] > longest[0]:
        longest[0] = row[0]
    if row[1] > longest[1]:
        longest[1] = row[1]
    if row[2] > longest[2]:
        longest[2] = row[2]
    if row[3] > longest[3]:
        longest[3] = row[3]
    if row[4] > longest[4]:
        longest[4] = row[4]
    if row[5] > longest[5]:
        longest[5] = row[5]

    last_number = number

    rolled = [one, two, three, four, five, six]
    std_dev = np.std(rolled)

    all_numbers = [one, two, three, four, five, six, errorcnt, rollnumber, std_dev, longest]

    return all_numbers, img_with_keypoints


while True:
    for i in range(3200):
                
        if (i > 3100):
            steptime = steptime + global_steptime * 0.1
        else:
            steptime = global_steptime    

        GPIO.output(4, GPIO.HIGH)
        time.sleep(steptime)
        GPIO.output(4, GPIO.LOW)
        time.sleep(steptime)
    
    time.sleep(0.3)    
    position_correct = False
  
    real_image,pos_img = get_images()
  
    while position_correct is not True:
        
        real_image,pos_img = get_images()
        
        M = cv2.moments(pos_img)

        #print(M)

        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        else:
            cX, cY = 0, 0
            
    # put text and highlight the center
        cv2.circle(pos_img, (cX, cY), 4, (0, 0, 0), -1)
        #cv2.putText(frame, "positioning", (cX - 25, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
        # Display the resulting frame
        
        
        if(cX < 135):
            GPIO.output(17, GPIO.HIGH)
            GPIO.output(4, GPIO.HIGH)
            time.sleep(global_steptime)
            GPIO.output(4, GPIO.LOW)
            time.sleep(global_steptime)      
        elif(cX >165):
            GPIO.output(17, GPIO.LOW)
            GPIO.output(4, GPIO.HIGH)
            time.sleep(global_steptime)
            GPIO.output(4, GPIO.LOW)
            time.sleep(global_steptime)
        else:
            position_correct = True
            print('correct position:')            
        print("X:", cX, "Y:", cY)

    cv2.imshow('newpos',pos_img)

    cv2.imshow('Input', real_image)
    processed_img = img_processing(real_image)
    numbers, keypoint_img = counting(processed_img)
    cv2.imshow('Output', keypoint_img)
    
    logging(numbers)

    if write_email is True and (numbers[7]%email_log_number) == 0:
        send_email(numbers)
    
    print('=================')    
    print('One: ', numbers[0])
    print('Two: ', numbers[1])
    print('Three: ', numbers[2])
    print('Four: ', numbers[3])
    print('Five: ', numbers[4])
    print('Six: ', numbers[5])
    print('Errors: ', numbers[6])
    print('All rolls: ', numbers[7])
    print('Deviation: ', numbers[8])
    print('=================')   

    cv2.waitKey()

cap.release()
cv2.destroyAllWindows()