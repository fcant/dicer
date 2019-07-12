import numpy as np
import cv2

GPIO.setwarnings(False)

GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)
GPIO.setup(4, GPIO.OUT)


brightness = 0

cap = cv2.VideoCapture(1)
cv2.VideoCapture.set(cap, 10, brightness)

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    ret, binary_image = cv2.threshold(gray, 230, 255, cv2.THRESH_BINARY)

    kernel = np.ones((5, 5), np.uint8)

    opening = cv2.morphologyEx(binary_image, cv2.MORPH_OPEN, kernel)
    binary_image = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)

    cv2.imshow('binary', binary_image)




    M = cv2.moments(binary_image)

    print(M)

    if M["m00"] != 0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
    else:
        cX, cY = 0, 0

    # put text and highlight the center
    cv2.circle(frame, (cX, cY), 5, (255, 0, 0), -1)
    cv2.putText(frame, "positioning", (cX - 25, cY - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    print("X:", cX, "Y:", cY)

    x=400
    y=360
    h=40
    w=70

    img_crop = binary_image[y:y + h, x:x + w]

    # Display the resulting frame
    cv2.imshow('frame',frame)

 
    if(cX < 100):
        GPIO.output(17, GPIO.HIGH)
        GPIO.output(4, GPIO.HIGH)
        time.sleep(0.05)
        GPIO.output(4, GPIO.LOW)
    elif(cX >300):
        GPIO.output(17, GPIO.LOW)
        GPIO.output(4, GPIO.HIGH)
        time.sleep(0.05)
        GPIO.output(4, GPIO.LOW)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break




# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()