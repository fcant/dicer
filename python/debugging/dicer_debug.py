# import threading

# import time
# from tkinter import *
# from PIL import ImageTk, Image
# import matplotlib.pyplot as plt
# from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
# from matplotlib.figure import Figure

# import datetime
import numpy as np
import cv2
import time
import smtplib
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart

darknumbers = False  # Dunkle Würfelaugen
gpios = False
dicer_ready = False

try:
    cap = cv2.VideoCapture(
        0)  # Bildquelle (Zahl ändern, falls mehrere Kameras angeschlossen sind (auch interne Webcams))
except cv2.error as e:
    print(e)
    pass

if not cap.isOpened():
    dicer_ready = False
    grey = cv2.imread('../dummy_image.png', 0)
    cv2.putText(grey, 'NO CAMERA', (10, 200), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
else:
    dicer_ready = True

global_steptime = 0.00015  # Abstand zwischen den Schritten

# Fleckendetektor konfigurieren
blob_params = cv2.SimpleBlobDetector_Params()
blob_params.filterByColor = True
blob_params.filterByArea = True
blob_params.minArea = 100
blob_params.filterByCircularity = False
blob_params.filterByInertia = True
blob_params.filterByConvexity = True

all_numbers = [0] * 12  # [one, two, three, four, five, six, errorcnt, rollnumber, std_dev, longest, row, last_number]

all_numbers[9] = [0] * 6  # 'longest' und 'row' enthalten jeweils 6 Werte
all_numbers[10] = [0] * 6


def get_images():
    for i in range(5):
        ret, frame = cap.read()

    if ret is not True:
        grey = cv2.imread('../grey.png', 0)
        #cv2.putText(grey, 'NO CAMERA', (10, 200), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
        pos_img = np.zeros(shape=[200, 200, 3], dtype=np.uint8)
    else:
        # Bildausschnitte von Würfel und Positionserkennung
        y = 160
        h = 240

        x = 220
        w = 240

        real_image = frame[y:y + h, x:x + w]
        grey = cv2.cvtColor(real_image, cv2.COLOR_BGR2GRAY)
        #cv2.imshow('input', grey)

        y = 115
        h = 20

        pos_img = frame[y:y + h, x:x + w]
        pos_img = cv2.cvtColor(pos_img, cv2.COLOR_BGR2GRAY)
        ret, pos_img = cv2.threshold(pos_img, 245, 255, cv2.THRESH_BINARY)
        cv2.imshow('pos', pos_img)
       #cv2.imwrite('grey.png',grey)
    return grey, pos_img


def hough_detector(input_image):
    # cv2.imshow('hough_input', input_image)
    img = cv2.medianBlur(input_image, 5)  # Bild gätten mit Gauß
    cimg = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)  # Farbraum umwandeln (nur für die farbigen Kreise)
    circles = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT, 1, 20, param1=220, param2=10, minRadius=10,
                               maxRadius=30)  # param1: Schwellenwert, param2: muss man ausprobieren

    h_number = 0

    try:  # Kreise zählen und markieren
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            # draw the outer circle
            cv2.circle(cimg, (i[0], i[1]), i[2], (0, 255, 0), 2)
            # draw the center of the circle
            # cv2.circle(cimg, (i[0], i[1]), 3, (0, 255, 50), 4)
            h_number += 1
    except:
        print('HOUGH DETECTOR ERROR, NO CIRCLES FOUND')

    cv2.putText(cimg, str(h_number), (10, 200), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 50), 2, cv2.LINE_AA)
    cv2.imshow('hough detector - Press Q to exit', cimg)

    return h_number


def img_processing(image_input):  # Bild vorbereitung
    # input_frame = cv2.cvtColor(image_input, cv2.COLOR_BGR2GRAY) #Kamerabild in Graustufen umwandeln

    ret, binary_image = cv2.threshold(image_input, 230, 255,
                                      cv2.THRESH_BINARY)  # Schwellenwertbild abspeichern

    if not darknumbers:
        binary_image = cv2.bitwise_not(binary_image)

    cv2.imwrite('binary.png', binary_image)
    # cv2.imshow('binary', binary_image)

    kernel_round = np.array([
                             [0, 0, 0, 1, 1, 1, 0, 0, 0],
                             [0, 1, 1, 1, 1, 1, 1, 1, 0],
                             [0, 1, 1, 1, 1, 1, 1, 1, 0],
                             [1, 1, 1, 1, 1, 1, 1, 1, 1],
                             [1, 1, 1, 1, 1, 1, 1, 1, 1],
                             [1, 1, 1, 1, 1, 1, 1, 1, 1],
                             [0, 1, 1, 1, 1, 1, 1, 1, 0],
                             [0, 1, 1, 1, 1, 1, 1, 1, 0],
                             [0, 0, 0, 1, 1, 1, 0, 0, 0]], dtype=np.uint8)  # Kreisförmige Maske erzeugen

    dilate = cv2.dilate(binary_image, kernel_round, iterations=3)  # Dilatation anwenden
    cv2.imwrite('dilate.png', dilate)

    erode = cv2.erode(dilate, kernel_round, iterations=2)  # Erosion anwenden
    cv2.imwrite('erode.png', erode)

    return erode


def counting(image, all_numbers):
    one = all_numbers[0]
    two = all_numbers[1]
    three = all_numbers[2]
    four = all_numbers[3]
    five = all_numbers[4]
    six = all_numbers[5]
    errorcnt = all_numbers[6]
    rollnumber = all_numbers[7]

    longest = all_numbers[9]
    row = all_numbers[10]
    last_number = all_numbers[11]

    detector = cv2.SimpleBlobDetector_create(blob_params)
    keypoints = detector.detect(image)
    img_with_keypoints = cv2.drawKeypoints(image, keypoints, np.array([]), (0, 0, 255),
                                           cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    blob_number = 0

    for i in keypoints[0:]:
        blob_number = blob_number + 1

    rollnumber += 1

    hough_number = hough_detector(image)

    if blob_number == hough_number:
        number = blob_number
        print('DETECTED: ', number)
        cv2.putText(img_with_keypoints, str(number), (10, 200), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2,
                    cv2.LINE_AA)

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
    else:
        print('NOT MATCHING FILTERS')
        errorcnt = errorcnt + 1
        cv2.imwrite('errors/' + str(errorcnt) + ' matching error.png', image)

    rolled = [one, two, three, four, five, six]
    std_dev = np.std(rolled)

    all_numbers[0] = one
    all_numbers[1] = two
    all_numbers[2] = three
    all_numbers[3] = four
    all_numbers[4] = five
    all_numbers[5] = six
    all_numbers[6] = errorcnt
    all_numbers[7] = rollnumber
    all_numbers[8] = std_dev
    all_numbers[9] = longest
    all_numbers[10] = row
    all_numbers[11] = last_number

    return all_numbers, img_with_keypoints


while True:

    real_image, pos_img = get_images()
    # cv2.imshow('pos', pos_img)

    #M = cv2.moments(pos_img)  # Schwerpunkt berechnen

    # print(M)

    #if M["m00"] != 0:
    #    cX = int(M["m10"] / M["m00"])
    #    cY = int(M["m01"] / M["m00"])
    #else:
    #    cX, cY = 0, 0

    # put text and highlight the center
    #cv2.circle(pos_img, (cX, cY), 4, (0, 0, 0), -1)
    # cv2.putText(frame, "positioning", (cX - 25, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
    # Display the resulting frame

    #print("X:", cX, "Y:", cY)
    #cv2.imshow('newpos', pos_img)

    processed_img = img_processing(real_image)
    numbers, blob_img = counting(processed_img, all_numbers)
    cv2.imshow('blob detector - Press Q to exit', blob_img)
    cv2.imshow('Input - Press Q to exit', real_image)

    if cv2.waitKey(300) & 0xFF == ord('q'):  # Q drücken, zum beenden
        break

cap.release()
cv2.destroyAllWindows()
