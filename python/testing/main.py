import cv2
import numpy as np


def pixel_counter(width, height):

    black =0

    for x in range(0,width-1):
        for y in range(0,height-1):
            if negativ[x,y] == 0:
                black = black + 1


    print("black:",black)
    if (black > 0) and (black < 900):
        print("pixel_counter: 1")
    elif (black > 1400) and (black < 1500):
        print("pixel_counter: 2")
    elif (black > 2200) and (black < 2500):
        print("pixel_counter: 3")
    elif (black > 2700) and (black < 2900):
        print("pixel_counter: 4")
    elif (black > 3500) and (black < 4800):
        print("pixel_counter: 5")
    elif (black > 4800):
        print("pixel_counter: 6")


cap = cv2.VideoCapture(0)

empty = cv2.imread('empty.png',0) # >0: 3 channel, =0: grau, <0: bild + alpha

params = cv2.SimpleBlobDetector_Params()

while(True):
    # Capture frame-by-frame
    ret, real_frame = cap.read() #ret gibt true oder false zurück, checkt ob video läuft

    frame = real_frame
    cap.set(10,0)
    # Our operations on the frame come here
    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #Kamerabild in Graustufen

    frame=grey

    cv2.absdiff(frame, empty, frame)  #mit leerer Hintergrundaufnahme subtrahieren




    blur = cv2.medianBlur(frame, 5)

    #output = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
    ret, output = cv2.threshold(frame, 90, 255, cv2.THRESH_BINARY) #Binärer Schwellenwert anwenden

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    elif ret == 0:
        break

    negativ = cv2.bitwise_not(output)


    height = output.shape[1] #y
    width = output.shape[0] #x
    #print('breite:', width)
    #print('höhe:', height)

    #pixel_counter(width, height)

    # Display the resulting frame
    #cv2.imshow('output', output)

    params.filterByColor = True
    params.filterByArea = True
    params.minArea = 100
    params.filterByCircularity = False
    params.filterByInertia = False
    params.filterByConvexity = True


    detector = cv2.SimpleBlobDetector_create(params)
    keypoints = detector.detect(negativ)

    img_with_keypoints = cv2.drawKeypoints(negativ, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    #print(img_with_keypoints.shape)
    #print(frame.shape)
    #print(grey.shape)


    #cv2.imshow("Press Q to close", img_with_keypoints);

    number = 0
    for i in keypoints[0:]:
        number = number + 1
    print(number)

    numpy_horizontal_concat = np.concatenate((real_frame, img_with_keypoints), axis=1)
    cv2.imshow('Press Q to close this', numpy_horizontal_concat)


# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()


