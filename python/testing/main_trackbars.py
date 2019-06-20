import cv2
import numpy as np

binary_value = 100 #startwerte
#trackbar_val =0

def binary(input_value):
    global binary_value
    binary_value = input_value
    print(input_value)

try:
    cap = cv2.VideoCapture(0)
except ValueError:
    print('Keine Kamera!')

empty = cv2.imread('empty.png',0) # >0: 3 channel, =0: grau, <0: bild + alpha

params = cv2.SimpleBlobDetector_Params()
params.filterByColor = True
params.filterByArea = True
params.minArea = 100
params.filterByCircularity = False
params.filterByInertia = True
params.filterByConvexity = True

main_window='OUTPUT - Press Q to close this'
cv2.namedWindow(main_window)
cv2.createTrackbar('Trackbar', main_window, binary_value, 255, binary)


while(True):
    # Capture frame-by-frame

    ret, real_frame = cap.read() # ret gibt true oder false zurück, checkt ob video läuft

    if ret == 0:
        cv2.imshow('INPUT_BINARY', binary_image)
        break

    input_frame = real_frame # umspeichern um das Originalbild zu behalten
    input_frame = cv2.cvtColor(input_frame, cv2.COLOR_BGR2GRAY) #Kamerabild in Graustufen

    cv2.absdiff(input_frame, empty, input_frame)  #mit leerer Hintergrundaufnahme subtrahieren

    cv2.imshow('INPUT', input_frame)

    ret, binary_image = cv2.threshold(input_frame, binary_value, 255, cv2.THRESH_BINARY)

    cv2.imshow('INPUT_BINARY', binary_image)

    #binary_image_neg = cv2.bitwise_not(binary_image)
    #binary_image_neg = cv2.medianBlur(binary_image_neg, 5)

    kernel = np.ones((5, 5), np.uint8)

    opening = cv2.morphologyEx(binary_image, cv2.MORPH_OPEN, kernel)
    closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)

    #erosion = cv2.dilate(binary_image, kernel, iterations=2)
    #cv2.imshow(main_window, erosion)

    cv2.imshow('Opening', opening)
    cv2.imshow('Closing', closing)

    closing_neg = cv2.bitwise_not(closing)

    detector = cv2.SimpleBlobDetector_create(params)
    keypoints = detector.detect(closing_neg)



    img_with_keypoints = cv2.drawKeypoints(closing_neg, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    number = 0
    for i in keypoints[0:]:
        number = number + 1
    print(number)

    cv2.imshow(main_window, img_with_keypoints)

    if cv2.waitKey(1) & 0xFF == ord('q'): # Q zum beenden
        break
    elif ret == 0:  # oder Aufnahme wird beendet
        break


# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()

