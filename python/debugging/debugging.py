import numpy as np
import cv2

cap = cv2.VideoCapture(0)

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
    w=220

    real_frame = real_frame[y:y + h, x:x + w] #zuschneiden

    input_frame = real_frame # umspeichern um das Originalbild zu behalten
    input_frame = cv2.cvtColor(input_frame, cv2.COLOR_BGR2GRAY) #Kamerabild in Graustufen umwandeln

    cv2.imshow('INPUT', input_frame) #anzeigen
    cv2.imwrite('INPUT.png', input_frame) #abspeichern

    ret, binary_image = cv2.threshold(input_frame, 180, 255, cv2.THRESH_BINARY) #Schwellenwertbild abspeichern

    cv2.imshow('INPUT_BINARY', binary_image)
    cv2.imwrite('INPUT_BINARY.png', binary_image)

    #kernel = np.ones((5, 5), np.uint8)

    #kernel_round = np.array([[0, 0, 1, 0, 0],[0,1,1,1,0],[1,1,1,1,1],[0,1,1,1,0],[0,0,1,0,0]], dtype=np.uint8)
    #opening1 = cv2.morphologyEx(binary_image, cv2.MORPH_OPEN, kernel_round)
    #erosion1 = cv2.erode(binary_image, kernel_round, iterations = 1)
    #cv2.imshow('KERNEL_ROUND', erosion1)

    #kernel_round = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(9,9))

    kernel_rect = np.ones((9, 9), np.uint8) #quadratische Maske erzeugen
    opening = cv2.morphologyEx(binary_image, cv2.MORPH_OPEN, kernel_rect) #Opening anwenden, um Rauschen zu entfernen
    
    cv2.imshow('opening', opening)
    cv2.imwrite('opening.png', opening)
    
    kernel_round = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(12,12)) #Ellipse als Maske erzeugen, un Punktförmigkeit der Augenzahlen beizubehalten
    erosion = cv2.erode(opening, kernel_round, iterations = 2) #zweimal Erosion anwenden

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
    dilate = cv2.dilate(erosion, kernel_round, iterations = 2)   #Dilatation anwenden, um weiße Punkte wieder zu vergrößern 
    
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

    if cv2.waitKey(1) & 0xFF == ord('q'): # Q zum beenden
        break
    elif ret == 0:  # oder Aufnahme wird beendet
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
    