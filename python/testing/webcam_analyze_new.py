
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

cap = cv2.VideoCapture(0)

while(True):
    # Capture frame-by-frame


    ret, frame = cap.read()

    # Our operations on the frame come here
    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    y=200
    h=230
    
    x=260
    w=250

    grey = grey[y:y + h, x:x + w]

    cv2.imshow('raw',grey)

    ret, binary_image = cv2.threshold(grey, 240, 255, cv2.THRESH_BINARY)

    cv2.imshow('binary',binary_image)

    kernel = np.ones((6, 6), np.uint8)

    opening = cv2.morphologyEx(binary_image, cv2.MORPH_OPEN, kernel)
    
    cv2.imshow('opening',opening)   
    
    #kernel = np.ones((4, 4), np.uint8)
    
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
    print(number)

    cv2.imshow('output',img_with_keypoints)
    
    

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()