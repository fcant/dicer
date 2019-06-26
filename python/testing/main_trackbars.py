import cv2
import numpy as np

dark_dice = True

def binary(input_value):
    global binary_value
    if input_value == 0:
        input_value = 1

    print(binary_value)
    binary_value = input_value


def brightness(input_value):
    global brightness_value
    brightness_value = input_value


def contrast(input_value):
    global contrast_value
    contrast_value = input_value


def saturation(input_value):
    global saturation_value
    saturation_value = input_value


def hue(input_value):
    global hue_value
    hue_value = input_value


def gain(input_value):
    global gain_value
    gain_value = input_value


def exposure(input_value):
    global exposure_value
    exposure_value = input_value


cap = cv2.VideoCapture(0)

#startwerte
binary_value = 100

brightness_value = int(cap.get(10))
contrast_value = int(cap.get(11))
saturation_value = int(cap.get(12))
#hue_value = int(cap.get(13))
#gain_value = 0
#exposure_value = 0

main_window='OUTPUT - Press Q to close this'
cv2.namedWindow(main_window)
error_image = cv2.imread('camera_error.png',1)



empty = cv2.imread('empty.png',0) # >0: 3 channel, =0: grau, <0: bild + alpha

blob_params = cv2.SimpleBlobDetector_Params()

blob_params.filterByColor = True
blob_params.filterByArea = True
blob_params.minArea = 100
blob_params.filterByCircularity = False
blob_params.filterByInertia = True
blob_params.filterByConvexity = True

cv2.createTrackbar('Binary', main_window, binary_value, 255, binary)
cv2.createTrackbar('Brightness', main_window, brightness_value, 255, brightness)
cv2.createTrackbar('Contrast', main_window, contrast_value, 255, contrast)
cv2.createTrackbar('Saturation', main_window, saturation_value, 255, saturation)
#cv2.createTrackbar('Hue', main_window, hue_value, 255, hue)
#cv2.createTrackbar('Gain', main_window, gain_value, 255, gain)
#cv2.createTrackbar('Exposure', main_window, 0, 5, exposure)

while(True):
    # Capture frame-by-frame

    cap.set(10, brightness_value)      #Brightness
    cap.set(11, contrast_value)    #Contrast
    cap.set(12, saturation_value)    #Saturation
    #cap.set(13, hue_value)    #Hue
#    cap.set(14, gain_value)    #Gain
    #cap.set(15, exposure_value)    #Exposure




    ret, real_frame = cap.read() # ret gibt true oder false zurück, checkt ob video läuft

    if ret == 0:
        cv2.imshow(main_window, error_image)
        cv2.waitKey()
        break

    input_frame = real_frame # umspeichern um das Originalbild zu behalten
    input_frame = cv2.cvtColor(input_frame, cv2.COLOR_BGR2GRAY) #Kamerabild in Graustufen

    cv2.imshow('INPUT', input_frame)

    cv2.absdiff(input_frame, empty, input_frame)  #mit leerer Hintergrundaufnahme subtrahieren

    ret, binary_image = cv2.threshold(input_frame, binary_value, 255, cv2.THRESH_BINARY)

    cv2.imshow('INPUT_BINARY', binary_image)

    #binary_image_neg = cv2.bitwise_not(binary_image)
    #binary_image_neg = cv2.medianBlur(binary_image_neg, 5)

    kernel = np.ones((5, 5), np.uint8)

    opening = cv2.morphologyEx(binary_image, cv2.MORPH_OPEN, kernel)
    closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)

    #erosion = cv2.dilate(binary_image, kernel, iterations=2)
    #cv2.imshow(main_window, erosion)

#    cv2.imshow('Opening', opening)
#    cv2.imshow('Closing', closing)

    if dark_dice == True:


        edges = cv2.Canny(closing, 100, 200)

        contours, hierarchy = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)




        if len(contours) > 0:
            cnt = contours[0]
        else:
            print("Sorry No contour Found.")

        x, y, w, h = cv2.boundingRect(cnt)
        cv2.rectangle(closing, (x, y), (x + w, y + h), (158, 255, 0), 2) ##################

        img_crop = closing#[y:y + h, x:x + w]

        w = img_crop.shape[1]  # y
        h = img_crop.shape[0]  # x

        mask = np.zeros((h + 2, w + 2), np.uint8)

        cv2.floodFill(img_crop, mask, (0, 0), 255);

    cv2.imshow('mask', closing)

    cv2.imshow('area', img_crop)

    closing_neg = cv2.bitwise_not(closing)

    detector = cv2.SimpleBlobDetector_create(blob_params)
    keypoints = detector.detect(img_crop)



    img_with_keypoints = cv2.drawKeypoints(img_crop, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
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

