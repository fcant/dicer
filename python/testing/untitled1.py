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

global_steptime = 0.00005  # Abstand zwischen den Schrittmotor schritten
anlauf = 200

GPIO.output(4, GPIO.HIGH)

