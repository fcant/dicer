import RPi.GPIO as GPIO
import time
import numpy as np
import cv2

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

cap = cv2.VideoCapture(0)

steptime = 0.0003

wurfzahl=0
one = 0
two=0
three=0
four=0
five=0
six=0

errorcnt = 1

while(True):
    
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


    for i in range(5):
    
        ret, frame = cap.read()

    # Our operations qon the frame come here
    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    y=200
    h=230
    
    x=260
    w=250

    grey = grey[y:y + h, x:x + w]

    cv2.imshow('raw',grey)

    ret, binary_image = cv2.threshold(grey, 230, 255, cv2.THRESH_BINARY)

    cv2.imshow('binary',binary_image)

    kernel = np.ones((6, 6), np.uint8)

    opening = cv2.morphologyEx(binary_image, cv2.MORPH_OPEN, kernel)
    #closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)

    closing = opening

    cv2.imshow('closing',closing)
    
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
    elif number == 2:    
        two = two +1
    elif number == 3:    
        three = three +1
    elif number == 4:    
        four = four +1
    elif number == 5:
        five = five +1
    elif number == 6:
        six = six + 1
    elif number > 6 or number < 1:
        cv2.imwrite(str(errorcnt) + 'Fehlerbild.png', img_with_keypoints)
        errorcnt = errorcnt + 1
        print('FEHLER')   
        if errorcnt == 100:
            errorcnt = 1
        wurfzahl = wurfzahl - 1 
    
    wurfzahl = wurfzahl + 1     

    print("=======================")
    print("Einz: ", one)
    print("Zwei: ", two)
    print("Drei: ", three)
    print("Vier: ", four)
    print("Fuenf: ", five)
    print("Sechs: ", six)
    print('Gesamt: ', wurfzahl)

    cv2.imshow('output',img_with_keypoints)


    if cv2.waitKey(100) & 0xFF == ord('q'):
        break
    

    
    
        
cap.release()
cv2.destroyAllWindows()



# When everything done, release the capture