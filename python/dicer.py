# import threading

# import time
# from tkinter import *
# from PIL import ImageTk, Image
# import matplotlib.pyplot as plt
# from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
# from matplotlib.figure import Figure

#import datetime
import numpy as np
import cv2
import time
import smtplib
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart

gpios = True

try:  # Wenn Programm nicht auf einem Raspberry läuft, GPIOS nicht benutzen
    import RPi.GPIO as GPIO
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(17, GPIO.OUT)
    GPIO.setup(4, GPIO.OUT)
    GPIO.setup(18, GPIO.IN, pull_up_down = GPIO.PUD_UP)
    
except ImportError:
    gpios = False
    print('WARNING - no GPIOS found')


###########################################################################################################################

darknumbers = False  # Dunkle Würfelaugen

send_email = True  # Email mit Messdaten versenden?
email_log_number = 3000  # Nach wie vielen Würfen soll eine Email geschrieben werden

error_logging = True



cap = cv2.VideoCapture(0)  # Bildquelle (Zahl ändern, falls mehrere Kameras angeschlossen sind (auch interne Webcams))

###########################################################################################################################

print('Setting up...')

interrupted = False
dicer_ready = False

ret, frame = cap.read() # Test, ob Kamera funktionert

if ret is not True: #Wenn Kamera nicht geht, Dummy Image laden
        dicer_ready = False
        grey = cv2.imread('dummy_image.png', 0)
        cv2.putText(grey, 'NO CAMERA', (10, 200), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
        pos_img = np.zeros(shape=[100, 100, 1], dtype=np.uint8)
        cv2.imshow('Press any key to exit', grey)
        cv2.waitKey()  # Taste drücken, zum beenden
else:
    dicer_ready = True

if GPIO.input(18) == 0:
    print('Temperature relay is offline, stopping')
    dicer_ready = False

global_steptime = 0.00015  # Abstand zwischen den Schritten

# Fleckendetektor konfigurieren
blob_params = cv2.SimpleBlobDetector_Params()
blob_params.filterByColor = True
blob_params.filterByArea = True
blob_params.minArea = 100
blob_params.filterByCircularity = False
blob_params.filterByInertia = True
blob_params.filterByConvexity = True

all_numbers = [0] * 9  # [one, two, three, four, five, six, errorcnt, rollnumber, std_dev


def interr(channel):
    global gpios
    global dicer_ready
    global interrupted
    gpios = False
    dicer_ready = False
    interrupted = True
    write_email
    print('Interrupt')

def step_plus():
    GPIO.output(17, GPIO.LOW)
    GPIO.output(4, GPIO.HIGH)
    time.sleep(global_steptime)
    GPIO.output(4, GPIO.LOW)
    time.sleep(global_steptime)
    GPIO.output(17, GPIO.LOW)


def step_minus():
    GPIO.output(17, GPIO.HIGH)
    GPIO.output(4, GPIO.HIGH)
    time.sleep(global_steptime)
    GPIO.output(4, GPIO.LOW)
    time.sleep(global_steptime)
    GPIO.output(17, GPIO.LOW)


def clock(now):
    time_seconds = int((time.time() - now))
    t_hr = int(time_seconds / 3600)
    t_min = int(time_seconds / 60) - (t_hr * 60)
    t_sec = int(time_seconds) - (t_min * 60)
    showTime = str(t_hr) + ':' + str(t_min).zfill(2)
    print(showTime)
    return showTime


def write_email(numbers, ctime, error):
    server = smtplib.SMTP('mail.gmx.net', 587)
    server.starttls()
    server.login('python-email@gmx.de', 'bojack123.')
    msg = MIMEMultipart()
    msg['From'] = 'python-email@gmx.de'
    msg['To'] = 'fabio.canterino@smail.th-koeln.de'
    if error:
        msg['Subject'] = 'Interrupt: Temperaturfehler'
    else:
        msg['Cc'] = 'anton.kraus@th-koeln.de'
        msg['Subject'] = 'Dicer - normaler Spielwürfel'
    message = str(numbers[0]) + ',' + str(numbers[1]) + ',' + str(numbers[2]) + ',' + str(numbers[3]) + ',' + str(
        numbers[4]) + ',' + str(numbers[5]) + ' Err: ' + str(numbers[6]) + ' All: ' + str(
        numbers[7]) + '\n' + 'Zeit: '+ str(ctime)
    msg.attach(MIMEText(message))

    server.send_message(msg)


def logging(numbers, ctime):

    file = open('log_plastik_seite2', 'w')
    file.write('Einz:' + str(numbers[0]) + '\n')
    file.write('Zwei:' + str(numbers[1]) + '\n')
    file.write("Drei: " + str(numbers[2]) + '\n')
    file.write("Vier: " + str(numbers[3]) + '\n')
    file.write("Fuenf: " + str(numbers[4]) + '\n')
    file.write("Sechs: " + str(numbers[5]) + '\n')
    file.write('Fehler: ' + str(numbers[6]) + '\n')
    file.write('Gesamt: ' + str(numbers[7]) + '\n')
    file.write('Standardabw: ' + str(numbers[8]) + '\n')
    file.write('Zeit: ' + str(ctime) + '\n')

    file.close()


def get_images():
    for i in range(5):
        ret, frame = cap.read()

    #cv2.imwrite('frame.png',frame)
    # Bildausschnitte von Würfel und Positionserkennung
    y = 160
    h = 240

    x = 220
    w = 240

    real_image = frame[y:y + h, x:x + w]
    grey = cv2.cvtColor(real_image, cv2.COLOR_BGR2GRAY)
    #cv2.imshow('input', grey)
    #cv2.imwrite('grey.png',grey)
    y = 115
    h = 20

    pos_img = frame[y:y + h, x:x + w]
    pos_img = cv2.cvtColor(pos_img, cv2.COLOR_BGR2GRAY)
    #cv2.imwrite('pos_raw.png',pos_img)
    ret, pos_img = cv2.threshold(pos_img, 245, 255, cv2.THRESH_BINARY)
    #cv2.imshow('pos', pos_img)
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
    cv2.imshow('hough detector', cimg)

    return h_number


def img_processing(image_input):  # Bild vorbereitung
    # input_frame = cv2.cvtColor(image_input, cv2.COLOR_BGR2GRAY) #Kamerabild in Graustufen umwandeln

    ret, binary_image = cv2.threshold(image_input, 230, 255,
                                      cv2.THRESH_BINARY)  # Schwellenwertbild abspeichern

    if not darknumbers:
        binary_image = cv2.bitwise_not(binary_image)

    # cv2.imshow('binary', binary_image)

    kernel_round = np.array([[0, 0, 0, 1, 1, 1, 0, 0, 0],
                             [0, 1, 1, 1, 1, 1, 1, 1, 0],
                             [0, 1, 1, 1, 1, 1, 1, 1, 0],
                             [1, 1, 1, 1, 1, 1, 1, 1, 1],
                             [1, 1, 1, 1, 1, 1, 1, 1, 1],
                             [1, 1, 1, 1, 1, 1, 1, 1, 1],
                             [0, 1, 1, 1, 1, 1, 1, 1, 0],
                             [0, 1, 1, 1, 1, 1, 1, 1, 0],
                             [0, 0, 0, 1, 1, 1, 0, 0, 0]], dtype=np.uint8)  # Kreisförmige Maske erzeugen

    dilate = cv2.dilate(binary_image, kernel_round, iterations=1)  # Dilatation anwenden

    erode = cv2.erode(dilate, kernel_round, iterations=1)  # Erosion anwenden

    return erode


def counting(image, all_numbers):
    one = all_numbers[0]
    two = all_numbers[1]
    three = all_numbers[2]
    four = all_numbers[3]
    five = all_numbers[4]
    six = all_numbers[5]
    errorcnt = all_numbers[6]
    success_rolls= all_numbers[7]

    detector = cv2.SimpleBlobDetector_create(blob_params)
    keypoints = detector.detect(image)
    img_with_keypoints = cv2.drawKeypoints(image, keypoints, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    blob_number = 0

    for i in keypoints[0:]:
        blob_number = blob_number + 1

    hough_number = hough_detector(image)

    if blob_number == hough_number:
        number = blob_number
        print('DETECTED: ', number)
        cv2.putText(img_with_keypoints, str(number), (10, 200), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2,
                    cv2.LINE_AA)
        
        
        if blob_number > 0 and blob_number < 7:
            raw_log = open('raw_numbers','a')
            raw_log.write(str(number) + '\n')
            raw_log.close()            
            success_rolls +=1
            all_numbers[number-1] += 1
        else:
            errorcnt = errorcnt + 1
            if error_logging is True:
                cv2.imwrite('errors/' + str(errorcnt) + ' number_error.png', image)


    else:
        print('NOT MATCHING FILTERS')
        errorcnt = errorcnt + 1
        if error_logging is True:
            cv2.imwrite('errors/' + str(errorcnt) + ' matching_error.png', image)

    rolled = [one, two, three, four, five, six]
    std_dev = np.std(rolled)

    all_numbers[6] = errorcnt
    all_numbers[7] = success_rolls
    all_numbers[8] = std_dev

    return all_numbers, img_with_keypoints



now = time.time()

if dicer_ready is True:
    GPIO.add_event_detect(18, GPIO.FALLING, callback = interr, bouncetime = 200)
    print('Starting...')


while dicer_ready is True:
    if gpios:
        for i in range(3200):

            if i > 3100:  # die letzten Schritte abbremsen
                steptime = steptime + global_steptime * 0.1
            else:
                steptime = global_steptime

            GPIO.output(4, GPIO.HIGH)
            time.sleep(steptime)
            GPIO.output(4, GPIO.LOW)
            time.sleep(steptime)

        time.sleep(0.6)  # Kurze Pause, damit Würfel ruhig liegen kann
    position_correct = False

    real_image, pos_img = get_images()  # Aufnahme machen

    while position_correct is not True and gpios is True:
        real_image, pos_img = get_images()
        #cv2.imshow('pos', pos_img)
    

        M = cv2.moments(pos_img)  # Schwerpunkt berechnen

        #print(M)

        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        else:
            cX, cY = 0, 0

        # put text and highlight the center
        cv2.circle(pos_img, (cX, cY), 4, (0, 0, 0), -1)
        # cv2.putText(frame, "positioning", (cX - 25, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
        # Display the resulting frame

        if cX < 115:
            GPIO.output(17, GPIO.HIGH)
            GPIO.output(4, GPIO.HIGH)
            time.sleep(global_steptime)
            GPIO.output(4, GPIO.LOW)
            time.sleep(global_steptime)
            GPIO.output(17, GPIO.LOW)
        elif cX > 135:
            GPIO.output(17, GPIO.LOW)
            GPIO.output(4, GPIO.HIGH)
            time.sleep(global_steptime)
            GPIO.output(4, GPIO.LOW)
            time.sleep(global_steptime)
        else:
            position_correct = True
            #print('correct position:')
        #print("X:", cX, "Y:", cY)
        #cv2.imwrite('newpos.png',pos_img)

    processed_img = img_processing(real_image)
    numbers, blob_img = counting(processed_img, all_numbers)
    cv2.imshow('blob detector', blob_img)
    cv2.imshow('Press Q to exit', real_image)

    ctime = clock(now)

    if (numbers[7] % 10) == 0:  # Nach 10 Messungen ins log schreiben
        logging(numbers, ctime)

    if send_email is True and (numbers[7] % email_log_number) == 0:
        write_email(numbers, ctime,0)

    print('=================')
    print('Time: ' + str(ctime))
    print('One: ', numbers[0])
    print('Two: ', numbers[1])
    print('Three: ', numbers[2])
    print('Four: ', numbers[3])
    print('Five: ', numbers[4])
    print('Six: ', numbers[5])
    print('Errors: ', numbers[6])
    print('Success rolls: ', numbers[7])
    print('Deviation: ', numbers[8])
    print('=================')


    #cv2.waitKey(100)
    if cv2.waitKey(300) & 0xFF == ord('q'):  # Q drücken, zum beenden
        break
    
if interrupted == True:
    write_email(numbers, ctime,1)

cap.release()
cv2.destroyAllWindows()
print('finished')
