import cv2
#import keyboard
#img1 = cv2.imread('flower.png',1)

#cv2.imshow('img',img1)
#cv2.waitKey()

cap = cv2.VideoCapture(0)

while(True):
    ret, frame = cap.read()
    cv2.imshow('frame', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()