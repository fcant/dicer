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

file = open('config', 'r')
old_values = file.readlines()
file.close()

one_min = int(old_values[3])
one_max = int(old_values[4])
two_min = int(old_values[6])
two_max = int(old_values[7])
three_min = int(old_values[9])
three_max = int(old_values[10])
four_min = int(old_values[12])
four_max = int(old_values[13])
five_min = int(old_values[15])
five_max = int(old_values[16])
six_min = int(old_values[18])
six_max = int(old_values[19])


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

    cv2.imshow('INPUT', input_frame) #anzeigen
    cv2.imwrite('INPUT.png', input_frame) #abspeichern

    ret, binary_image = cv2.threshold(input_frame, 200, 255, cv2.THRESH_BINARY) #Schwellenwertbild abspeichern

    cv2.imshow('INPUT_BINARY', binary_image)
    cv2.imwrite('INPUT_BINARY.png', binary_image)

    kernel_rect = np.ones((7, 7), np.uint8) #quadratische Maske erzeugen
    opening = cv2.morphologyEx(binary_image, cv2.MORPH_OPEN, kernel_rect)

    dark_numbers = False
    
    if dark_numbers == True:
                  
        cv2.imshow('opening', opening) 
       
        w = opening.shape[1]  # y
        h = opening.shape[0]  # x

        mask = np.zeros((h + 2, w + 2), np.uint8)
    
        cv2.floodFill(opening, mask, (0, 0), 255);
        cv2.floodFill(opening, mask, (0, 200), 255)
        
        cv2.imshow('flood', opening)        
    
        #kernel_rect = np.ones((9, 9), np.uint8) #quadratische Maske erzeugen
        #closing = cv2.morphologyEx(binary_image, cv2.MORPH_CLOSE, kernel_rect)   
    
    else:   
        opening = cv2.bitwise_not(opening)


    #kernel = np.ones((5, 5), np.uint8)

    #kernel_round = np.array([[0, 0, 1, 0, 0],[0,1,1,1,0],[1,1,1,1,1],[0,1,1,1,0],[0,0,1,0,0]], dtype=np.uint8)
    #opening1 = cv2.morphologyEx(binary_image, cv2.MORPH_OPEN, kernel_round)
    #erosion1 = cv2.erode(binary_image, kernel_round, iterations = 1)
    #cv2.imshow('KERNEL_ROUND', erosion1)

    #kernel_round = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(9,9))


    cv2.imshow('bit_not', opening)  
  
    kernel_round = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(12,12)) #Ellipse als Maske erzeugen, un Punktförmigkeit der Augenzahlen beizubehalten
    erosion = cv2.dilate(opening, kernel_round, iterations = 1) #zweimal Erosion anwenden

    cv2.imshow('Erosion', erosion)
    cv2.imwrite('Erosion.png', erosion)
    
    #kernel_rect = np.ones((5, 5), np.uint8)
    
    kernel_round = np.array([[0,0,0,1,1,1,0,0,0],
                             [0,1,1,1,1,1,1,1,0],
                             [0,1,1,1,1,1,1,1,0],
                             [1,1,1,1,1,1,1,1,1],
                             [1,1,1,1,1,1,1,1,1],
                             [1,1,1,1,1,1,1,1,1],
                             [0,1,1,1,1,1,1,1,0],
                             [0,1,1,1,1,1,1,1,0],
                             [0,0,0,1,1,1,0,0,0]], dtype=np.uint8) #Kreisförmige Maske erzeugen
    
    #kernel_round = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(12,12))
    dilate = cv2.erode(erosion, kernel_round, iterations = 1)   #Dilatation anwenden, um weiße Punkte wieder zu vergrößern 
    
    cv2.imshow('DILATE', dilate)
    cv2.imwrite('dilate.png', dilate)
    
    #closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)

  #  closing =  erosion
#    closing = cv2.bitwise_not(closing)

#    w = closing.shape[1]  # y
#    h = closing.shape[0]  # x

#    mask = np.zeros((h + 2, w + 2), np.uint8)
    
#    cv2.floodFill(closing, mask, (0, 0), 255);
#    cv2.floodFill(closing, mask, (0, 200), 255)

    #dilate = cv2.bitwise_not(dilate)

    detector = cv2.SimpleBlobDetector_create(blob_params)
    keypoints = detector.detect(dilate)
    img_with_keypoints = cv2.drawKeypoints(dilate, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
 
    cv2.imshow('keypoints', img_with_keypoints)
    
  #  for x in range(230):
  #      for y in range(230):
  #          if dilate[x,y] == 0:
  #              schwarz =+ 1




    if cv2.waitKey(1) & 0xFF == ord('q'): # Q zum beenden
        break
    elif ret == 0:  # oder Aufnahme wird beendet
        break


        

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
    