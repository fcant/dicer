import threading
import datetime
import time
import numpy as np
import cv2
from tkinter import *
import RPi.GPIO as GPIO
import time
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import smtplib
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart


# Raspberry IOs initialisieren
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)
GPIO.setup(4, GPIO.OUT)

steptime = 0.0001  # Abstand zwischen den Schrittmotor schritten
anlauf = 200

while True:
    for i in range(3200):

        GPIO.output(4, GPIO.HIGH)
        time.sleep(steptime)
        GPIO.output(4, GPIO.LOW)
        time.sleep(steptime)
    time.sleep(0.5)



