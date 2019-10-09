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

cap = cv2.VideoCapture(0)

global_steptime = 0.0001

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)
GPIO.setup(4, GPIO.OUT)

def step_plus():
    GPIO.output(17, GPIO.LOW)
    GPIO.output(4, GPIO.HIGH)
    time.sleep(global_steptime)
    GPIO.output(4, GPIO.LOW)
    time.sleep(global_steptime)
    print('step')


def step_minus():
    GPIO.output(17, GPIO.HIGH)
    GPIO.output(4, GPIO.HIGH)
    time.sleep(global_steptime)
    GPIO.output(4, GPIO.LOW)
    time.sleep(global_steptime)
    GPIO.output(17, GPIO.LOW)
    print('step')


while True:
    for i in range(3200):
                
        if (i > 2900):
            steptime = steptime + global_steptime * 0.1
        else:
            steptime = global_steptime    

        GPIO.output(4, GPIO.HIGH)
        time.sleep(steptime)
        GPIO.output(4, GPIO.LOW)
        time.sleep(steptime)
    
    time.sleep(0.3)    
    position_correct = False
    
    while position_correct is not True:
        for i in range(5):
            ret, frame = cap.read()

        y = 430
        h = 20

        x = 220
        w = 280
        
        pos_img = frame[y:y + h, x:x + w]
        pos_img2 = cv2.cvtColor(pos_img, cv2.COLOR_BGR2GRAY)
        ret, pos_img2 = cv2.threshold(pos_img2, 220, 255,cv2.THRESH_BINARY)  # Schwellenwertbild abspeichern
        
        M = cv2.moments(pos_img2)

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
        elif(cX >145):
            GPIO.output(17, GPIO.LOW)
            GPIO.output(4, GPIO.HIGH)
            time.sleep(global_steptime)
            GPIO.output(4, GPIO.LOW)
            time.sleep(global_steptime)
        else:
            position_correct = True
            print('correct:')            
        print("X:", cX, "Y:", cY)
        cv2.imshow('newpos',pos_img)
      
    y = 130
    h = 280

    x = 220
    w = 280

    frame1 = frame[y:y + h, x:x + w]
    grey = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
        
    cv2.imshow('output',grey)

    cv2.waitKey()
    
