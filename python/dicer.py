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

#################################################################################################################

global_steptime = 0.0001  # Abstand zwischen den Schrittmotor schritten
write_email = True  # Email mit Messdaten versenden?
email_log_number = 5000  # Nach wie vielen Würfen soll eine Email geschrieben werden

# Emailserver konfigurieren und starten
#server = smtplib.SMTP('mail.gmx.net', 587)
#server.starttls()

cap = cv2.VideoCapture(0)  # Videoquelle initialisieren, Zahl gibt Videoquelle an

#################################################################################################################

# Tkinter initialisieren (GUI wird am Ende des Programms erstellt)



root = Tk()
root.title('Dicer V0.7')
#root.wm_iconbitmap('2dice.ico')
root.minsize(1250,300)
# Icons made by "https://www.flaticon.com/authors/freepik" from https://www.flaticon.com/

# Raspberry IOs initialisieren
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)
GPIO.setup(4, GPIO.OUT)

# Fleckendetektor konfigurieren
blob_params = cv2.SimpleBlobDetector_Params()
blob_params.filterByColor = True
blob_params.filterByArea = True
blob_params.minArea = 100
blob_params.filterByCircularity = False
blob_params.filterByInertia = True
blob_params.filterByConvexity = True

rollnumber = 0
one = 0
two = 0
three = 0
four = 0
five = 0
six = 0

ready = 0
errorcnt = 0

file = open('config', 'r')
config_values = file.readlines()
file.close()

#one_min = int(config_values[3])
#one_max = int(config_values[4])
#two_min = int(config_values[6])
#two_max = int(config_values[7])
#three_min = int(config_values[9])
#three_max = int(config_values[10])
#four_min = int(config_values[12])
#four_max = int(config_values[13])
#five_min = int(config_values[15])
#five_max = int(config_values[16])
#six_min = int(config_values[18])
#six_max = int(config_values[19])

clock_run = False  # Uhr an und aus

start_stop = False  # Gibt an, ob Messung gestartet wurde

img_proc_run = False  # Gibt an, ob gerade ein Bild ausgewertet wird
stepper_run = False  # Gibt an, ob der Motor schon läuft
liveview_running = False  # Gibt an, ob LiveView schon läuft damit kein zweiter Thread gestartet wird


# ret, frame = cap.read() #ret gibt true oder false zurück, checkt ob video läuft

class StepperThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)

    def run(self):
        global stepper_run
        global img_proc_run
        # Wenn Motor freigegeben ist und kein Bild aufgenommen wird:
        stepper()
        img_proc_run = True  # Gibt die Bildaufnahme und Verarbeitung frei
        stepper_run = False  # Motor fertig gedreht


class LiveView(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self)

    def run(self):
        global liveview_running
        liveview_running = True
        grey = get_image()
        show_raw(grey)
        processed_img = img_processing(grey)
        numbers,keypoint_img = counting(processed_img)
        show_output(keypoint_img)
        liveview_running = False


def stepper():
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


class ClockThread(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self)

    def run(self):
        now = time.time()
        while start_stop:
            clock(now)


def clock(now):
    time_seconds = int((time.time() - now))
    t_hr = int(time_seconds / 3600)
    t_min = int(time_seconds / 60) - (t_hr * 60)
    t_sec = int(time_seconds) - (t_min * 60)
    showTime = str(t_hr).zfill(3), ':', str(t_min).zfill(2), ':', str(t_sec).zfill(2)
    timer.config(text=showTime)


def reset_log():
    global rollnumber
    global one
    global two
    global three
    global four
    global five
    global six
    global errorcnt

    rollnumber = 0
    one = 0
    two = 0
    three = 0
    four = 0
    five = 0
    six = 0
    errorcnt = 0


def reset_calibration():
    global one_min
    global one_max
    global two_min
    global two_max
    global three_min
    global three_max
    global four_min
    global four_max
    global five_min
    global five_max
    global six_min
    global six_max

    one_min = 9999
    one_max = 0
    two_min = 9999
    two_max = 0
    three_min = 9999
    three_max = 0
    four_min = 9999
    four_max = 0
    five_min = 9999
    five_max = 0
    six_min = 9999
    six_max = 0

    config_values[3] = str(one_min) + '\n'
    config_values[4] = str(one_max) + '\n'
    config_values[6] = str(two_min) + '\n'
    config_values[7] = str(two_max) + '\n'
    config_values[9] = str(three_min) + '\n'
    config_values[10] = str(three_max) + '\n'
    config_values[12] = str(four_min) + '\n'
    config_values[13] = str(four_max) + '\n'
    config_values[15] = str(five_min) + '\n'
    config_values[16] = str(five_max) + '\n'
    config_values[18] = str(six_min) + '\n'
    config_values[19] = str(six_max) + '\n'

    file3 = open('config', 'w')
    file3.writelines(config_values)
    file3.close()


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


def get_image():
    for i in range(5):
        ret, frame = cap.read()

    if not ret:
        grey = cv2.imread('dummy_image.png', 0)
        # grey = cv2.imread('INPUT.png', 0)
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(grey, 'NO CAMERA', (10, 200), font, 1, (255, 255, 255), 2, cv2.LINE_AA)
    else:
        y = 170
        h = 280

        x = 220
        w = 280

        frame1 = frame[y:y + h, x:x + w]
        grey = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)

    return grey


def send_email(numbers):
    timestamp = timer.cget("text")

    server.login('python-email@gmx.de', 'bojack123.')

    msg = MIMEMultipart()
    msg['From'] = 'python-email@gmx.de'
    msg['To'] = 'zug209@gmx.net'
    msg['Subject'] = 'Dicer update'
    message = str(numbers[0]) + ',' + str(numbers[1]) + ',' + str(numbers[2]) + ',' + str(numbers[3]) + ',' + str(
        numbers[4]) + ',' + str(numbers[5]) + ' Err: ' + str(numbers[6]) + ' All: ' + str(numbers[6]) + ' Std: ' + str(
        numbers[7]) + ' Time: ' + str(timestamp)
    msg.attach(MIMEText(message))

    server.send_message(msg)


def img_processing(image_input):
    # input_frame = cv2.cvtColor(image_input, cv2.COLOR_BGR2GRAY) #Kamerabild in Graustufen umwandeln

    ret, binary_image = cv2.threshold(image_input, binary_slider.get(), 255,
                                      cv2.THRESH_BINARY)  # Schwellenwertbild abspeichern

    if dark_numbers.get() == 1:

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

    kernel_round = np.array([[0, 0, 0, 1, 1, 1, 0, 0, 0],
                             [0, 1, 1, 1, 1, 1, 1, 1, 0],
                             [0, 1, 1, 1, 1, 1, 1, 1, 0],
                             [1, 1, 1, 1, 1, 1, 1, 1, 1],
                             [1, 1, 1, 1, 1, 1, 1, 1, 1],
                             [1, 1, 1, 1, 1, 1, 1, 1, 1],
                             [0, 1, 1, 1, 1, 1, 1, 1, 0],
                             [0, 1, 1, 1, 1, 1, 1, 1, 0],
                             [0, 0, 0, 1, 1, 1, 0, 0, 0]], dtype=np.uint8)  # Kreisförmige Maske erzeugen

    dilate = cv2.dilate(clean_eyes, kernel_round, iterations=2)  # zweimal Erosion anwenden

    erode = cv2.erode(dilate, kernel_round, iterations=1)  # zweimal Erosion anwenden

    return erode


def counting(image):
    global rollnumber, b_pixels
    global one
    global two
    global three
    global four
    global five
    global six
    global errorcnt
    global config_values

    one_min = int(config_values[3])
    one_max = int(config_values[4])
    two_min = int(config_values[6])
    two_max = int(config_values[7])
    three_min = int(config_values[9])
    three_max = int(config_values[10])
    four_min = int(config_values[12])
    four_max = int(config_values[13])
    five_min = int(config_values[15])
    five_max = int(config_values[16])
    six_min = int(config_values[18])
    six_max = int(config_values[19])

    detector = cv2.SimpleBlobDetector_create(blob_params)
    keypoints = detector.detect(image)
    img_with_keypoints = cv2.drawKeypoints(image, keypoints, np.array([]), (0, 0, 255),
                                           cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    number = 0

    for i in keypoints[0:]:
        number = number + 1
    if start_stop:
        rollnumber += 1

    detected_number.config(text=str(number))

    if not stepper_run:
        b_pixels = 0
        for x in range(0, 229):
            for y in range(0, 229):
                if image[x, y] == 0:
                    b_pixels += 1

        # print(number, ': ', b_pixels)

    if calibrating.get() == 1:
        if number == 1 and b_pixels < one_min:
            one_min = b_pixels - 50
            one += 1
        elif number == 1 and b_pixels > one_max:
            one_max = b_pixels + 50
            one += 1

        elif number == 2 and b_pixels < two_min:
            two_min = b_pixels - 50
            two += 1
        elif number == 2 and b_pixels > two_max:
            two_max = b_pixels + 50
            two += 1

        elif number == 3 and b_pixels < three_min:
            three_min = b_pixels - 50
            three += 1
        elif number == 3 and b_pixels > three_max:
            three_max = b_pixels + 50
            three += 1

        elif number == 4 and b_pixels < four_min:
            four_min = b_pixels - 50
            four += 1
        elif number == 4 and b_pixels > four_max:
            four_max = b_pixels + 50
            four += 1

        elif number == 5 and b_pixels < five_min:
            five_min = b_pixels - 50
            five += 1
        elif number == 5 and b_pixels > five_max:
            five_max = b_pixels + 50
            five += 1

        elif number == 6 and b_pixels < six_min:
            six_min = b_pixels - 50
            six += 1
        elif number == 6 and b_pixels > six_max:
            six_max = b_pixels + 50
            six += 1

        config_values[3] = str(one_min) + '\n'
        config_values[4] = str(one_max) + '\n'
        config_values[6] = str(two_min) + '\n'
        config_values[7] = str(two_max) + '\n'
        config_values[9] = str(three_min) + '\n'
        config_values[10] = str(three_max) + '\n'
        config_values[12] = str(four_min) + '\n'
        config_values[13] = str(four_max) + '\n'
        config_values[15] = str(five_min) + '\n'
        config_values[16] = str(five_max) + '\n'
        config_values[18] = str(six_min) + '\n'
        config_values[19] = str(six_max) + '\n'

        file1 = open('config', 'w')
        file1.writelines(config_values)
        file1.close()
    elif start_stop is True and img_proc_run is True:
        if number == 1:  # and (one_min < b_pixels < one_max):
            one += 1
        elif number == 2:  # and (two_min < b_pixels < two_max):
            two += 1
        elif number == 3:  # and (three_min < b_pixels < three_max):
            three += 1
        elif number == 4:  # and (four_min < b_pixels < four_max):
            four += 1
        elif number == 5:  # and (five_min < b_pixels < five_max):
            five += 1
        elif number == 6:  # and (six_min < b_pixels < six_max):
            six += 1
        else:
            errorcnt = errorcnt + 1
            if error_logging.get() == 1:
                cv2.imwrite('errors/' + str(errorcnt) + ' error.png', image)

    rolled = [one, two, three, four, five, six]
    std = int(np.std(rolled))

    all_numbers = [one, two, three, four, five, six, errorcnt, rollnumber, std]
    return all_numbers, img_with_keypoints


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
    file.write('Standardabw: ' + str(numbers[8]) + '\n')
    file.close()

    std_dev.config(text=str(numbers[8]))
    all_rolls.config(text=str(numbers[7]))
    errors.config(text=str(numbers[6]))

    values = [numbers[0], numbers[1], numbers[2], numbers[3], numbers[4], numbers[5]]

    if write_email is True and numbers[7] == email_log_number:
        send_email(numbers)

    ax.cla()
    ax.set_xlabel('Number')
    ax.set_ylabel('Count')
    ax.bar([1, 2, 3, 4, 5, 6], values)

    statistics_widget.draw()


def show_raw(image):
    if bin_true.get() == 1:
        ret, image = cv2.threshold(image, int(binary_slider.get()), 255, cv2.THRESH_BINARY)

    cv2image1 = cv2.cvtColor(image, cv2.COLOR_BGR2RGBA)
    img1 = Image.fromarray(cv2image1)
    imgtk1 = ImageTk.PhotoImage(image=img1)
    label_raw_image.imgtk = imgtk1
    label_raw_image.configure(image=imgtk1)
    label_raw_image.image = img1


def show_output(image):
    cv2image1 = cv2.cvtColor(image, cv2.COLOR_BGR2RGBA)
    img1 = Image.fromarray(cv2image1)
    imgtk1 = ImageTk.PhotoImage(image=img1)
    label_output_image.imgtk = imgtk1
    label_output_image.configure(image=imgtk1)
    label_output_image.image = img1


def save_image():
    raw = cv2.imread('last_raw.png')
    ts = datetime.datetime.now().timestamp()
    cv2.imwrite(str(ts) + '.png', raw)


def start_stop_prog():
    global start_stop

    if start_stop:
        start_stop = False
        start_stop_button.config(text='Start')
    else:
        start_stop = True
        threadc = ClockThread()
        threadc.start()
        start_stop_button.config(text='Stop')


def mainprogram():
    global stepper_run
    global img_proc_run
    global start_stop
    global config_values

    config_values[1] = str(binary_slider.get()) + '\n'

    conf_file = open('config', 'w')
    conf_file.writelines(config_values)
    conf_file.close()

    if start_stop is True and liveview_running is False:  # Start Button gedrückt und LiveView Prozess fertig?
        if stepper_run is False and img_proc_run is False:
            stepper_run = True
            MotorThread = StepperThread()
            MotorThread.start()
        if stepper_run is False and img_proc_run is True:  # Motor fertig gedreht und Bildfreigabe von Motor erhalten?
            raw_image = get_image()
            show_raw(raw_image)
            processed_img = img_processing(raw_image)
            numbers, keypoint_img = counting(processed_img)
            show_output(keypoint_img)
            logging(numbers)
            img_proc_run = False  # Bild wurde aufgenommen, Motor kann wieder drehen

    elif not start_stop:
        if liveview_running is False:  # live View
            thread2 = LiveView()
            thread2.start()

    root.after(100, mainprogram)


topFrame = Frame(root)
topFrame.pack(side='top', fill='x')

# Frame für aufgenommene Bilder:
image_Frame = Frame(topFrame)
image_Frame.pack(side='left')

# Platzhalter Bild laden und anzeigen:
# dummy_image = PhotoImage(file='dummy_image.png')

label_raw_image = Label(image_Frame)
label_raw_image.pack(side='top')

label_output_image = Label(image_Frame)
label_output_image.pack(side='bottom')

# Frame für detekt. Zahl und Graphen:
numbers_Frame = Frame(topFrame)
numbers_Frame.pack(side='left')

# Detektierte Zahl:
detected_number = Label(numbers_Frame, text='0', padx=50, pady=50)
detected_number.config(font=("Courier", 44))
detected_number.pack(side='left')

# Balkendiagramm erstellen (figure) und anzeigen (widget):
statistics_fig = Figure()
ax = statistics_fig.add_subplot(111)
ax.set_xlabel('Numbers')
ax.set_ylabel('Count')
statistics_widget = FigureCanvasTkAgg(statistics_fig, numbers_Frame)
statistics_widget.get_tk_widget().pack(side='right')
statistics_widget.draw()

# Frame für zählen, Berechnungen, Timer:
stat_Frame = Frame(topFrame)
stat_Frame.pack(side='left', padx=10)

# Start/Stop Button:
start_stop_button = Button(stat_Frame, text='Start', command=start_stop_prog)
start_stop_button.config(font=("Courier", 20))
start_stop_button.grid(row=0, column=5, pady=20, sticky=N)

# Auswertungen anzeigen:
Label(stat_Frame, text='All rolls: ').grid(row=1, column=5, sticky=W)
all_rolls = Label(stat_Frame, text='0')
all_rolls.grid(row=1, column=6, sticky=W, padx=10)

Label(stat_Frame, text='Errors: ').grid(row=2, column=5, sticky=W)
errors = Label(stat_Frame, text='0')
errors.grid(row=2, column=6, sticky=W, padx=10)

Label(stat_Frame, text='Standard deviation: ').grid(row=3, column=5, sticky=W)
std_dev = Label(stat_Frame, text='0')
std_dev.grid(row=3, column=6, sticky=W, padx=10)

Label(stat_Frame, text='irgendwas: ').grid(row=4, column=5, sticky=W)
errors = Label(stat_Frame, text='0')
errors.grid(row=4, column=6, sticky=W, padx=10)

Label(stat_Frame, text='irgendwas: ').grid(row=5, column=5, sticky=W)
errors = Label(stat_Frame, text='0')
errors.grid(row=5, column=6, sticky=W, padx=10)

# Timer:
Label(stat_Frame, text='Time: ').grid(row=7, column=5, sticky=W, pady=20)

timer = Label(stat_Frame, text='000 : 00 : 00')
timer.config(font=("Courier", 15))
timer.grid(row=8, column=5)

Button(stat_Frame, text='Reset log', command=reset_log).grid(row=10, column=5, rowspan=2, pady=50, sticky=S)

bottomFrame = Frame(root)
bottomFrame.pack(side=BOTTOM)

calibrating = IntVar()
error_logging = IntVar()
bin_true = IntVar()
dark_numbers = IntVar()

Checkbutton(bottomFrame, text="binary", variable=bin_true).grid(row=0, column=0)
Checkbutton(bottomFrame, text="Error logging", variable=error_logging).grid(row=0, column=2)
Checkbutton(bottomFrame, text="dark numbers", variable=dark_numbers).grid(row=0, column=3)
Checkbutton(bottomFrame, text="calibrating", variable=calibrating).grid(row=0, column=4)

Label(bottomFrame, text='Motor Control: ').grid(row=0, column=9, sticky=E, padx=20)
Button(bottomFrame, text='Step left', command=step_plus).grid(row=0, column=10)
Button(bottomFrame, text='Step right', command=step_minus).grid(row=0, column=11)

Label(bottomFrame, text='Binary value: ').grid(row=2, column=0, sticky=E, padx=10)
Button(bottomFrame, text='-', command=slider_minus).grid(row=2, column=1, sticky=E)
binary_slider = Scale(bottomFrame, from_=0, to=255, orient=HORIZONTAL)
binary_slider.grid(row=2, column=2)
binary_slider.set(int(config_values[1]))
Button(bottomFrame, text='+', command=slider_plus).grid(row=2, column=3, sticky=W)

Button(bottomFrame, text='Reset calibration', command=reset_calibration).grid(row=2, column=6)

mainprogram()
root.mainloop()
