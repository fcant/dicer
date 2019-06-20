import cv2

cap = cv2.VideoCapture(0)

# Capture frame-by-frame
ret, frame = cap.read() #ret gibt true oder false zurück, checkt ob video läuft
grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

#cv2.imwrite("empty.png", frame)
cv2.imwrite("dice_side.png", grey)

cap.release()
cv2.destroyAllWindows()


