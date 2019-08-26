import numpy as np
import cv2
import threading

cap = cv2.VideoCapture(0)

blob_params = cv2.SimpleBlobDetector_Params()

blob_params.filterByColor = True
blob_params.filterByArea = True
blob_params.minArea = 100
blob_params.filterByCircularity = False
blob_params.filterByInertia = True
blob_params.filterByConvexity = True

#binary_slider.set(int(old_values[1]))

#ret, empty_frame = cap.read() # ret gibt true oder false zurück, checkt ob video läuft
       
#y=200
#h=230
    
#x=280
#w=220

#empty_frame = empty_frame[y:y + h, x:x + w] 

#empty_frame = cv2.cvtColor(empty_frame, cv2.COLOR_BGR2GRAY) #Kamerabild in Graustufen

#cv2.imshow('INPUT_EMPTY', empty_frame) #Aufgenommenes Bild anzeigen


while(True):
    ret, real_frame = cap.read() # ret gibt true oder false zurück, checkt ob video läuft
       
    y=200
    h=230
    
    x=280
    w=230

    real_frame = real_frame[y:y + h, x:x + w] #zuschneiden



    input_frame = real_frame # umspeichern um das Originalbild zu behalten
    input_frame = cv2.cvtColor(input_frame, cv2.COLOR_BGR2GRAY) #Kamerabild in Graustufen umwandeln

    #input_frame = cv2.imread('INPUT2.png')

    #cv2.imshow('INPUT', input_frame) #anzeigen

    ret, binary_image = cv2.threshold(input_frame, 210, 255, cv2.THRESH_BINARY) #Schwellenwertbild abspeichern

    cv2.imshow('INPUT_BINARY', binary_image)

    dark_numbers = False
    
    if dark_numbers == True:
       
        w = binary_image.shape[1]  # y
        h = binary_image.shape[0]  # x

        mask = np.zeros((h + 2, w + 2), np.uint8)
    
        cv2.floodFill(binary_image, mask, (0, 0), 255);
        cv2.floodFill(binary_image, mask, (0, 200), 255)
        
        cv2.imshow('flood', binary_image)
 
        kernel_rect = np.ones((5, 5), np.uint8) #quadratische Maske erzeugen
 
        clean_eyes = cv2.erode(binary_image, kernel_rect, iterations = 1) #zweimal Erosion anwenden
        cv2.imshow('erode2', clean_eyes) #Erosion gegen weißes rauschen auf den schwarzen Augenzahlen
        
    
    else:   
        binary_image = cv2.bitwise_not(binary_image)
        
        cv2.imshow('NEGATIV', binary_image)
        
        clean_eyes = binary_image
  
  
 
    
    kernel_round = np.array([[0,0,0,1,1,1,0,0,0],
                             [0,1,1,1,1,1,1,1,0],
                             [0,1,1,1,1,1,1,1,0],
                             [1,1,1,1,1,1,1,1,1],
                             [1,1,1,1,1,1,1,1,1],
                             [1,1,1,1,1,1,1,1,1],
                             [0,1,1,1,1,1,1,1,0],
                             [0,1,1,1,1,1,1,1,0],
                             [0,0,0,1,1,1,0,0,0]], dtype=np.uint8) #Kreisförmige Maske erzeugen
    
    
  
  
  
    dilate = cv2.dilate(clean_eyes, kernel_round, iterations = 2) #zweimal Erosion anwenden
    cv2.imshow('DILATE', dilate)
    cv2.imwrite('DILATE', dilate)
    
    
    erode = cv2.erode(dilate, kernel_round, iterations = 1) #zweimal Erosion anwenden
    cv2.imshow('ERODE', erode) 
    
    
    detector = cv2.SimpleBlobDetector_create(blob_params)
    keypoints = detector.detect(erode)
    img_with_keypoints = cv2.drawKeypoints(erode, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
 
    cv2.imshow('keypoints', img_with_keypoints)



    if cv2.waitKey(1) & 0xFF == ord('q'): # Q zum beenden
        break
    elif ret == 0:  # oder Aufnahme wird beendet
        break


        

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
    