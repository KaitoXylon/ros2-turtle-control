import cv2
import os

cap = cv2.VideoCapture(0)

i = 0 

while True:
    ret , frame  = cap.read()
    if not ret:
        break

    cv2.imshow("calibration", frame)

    key = cv2.waitKey(1) 

    if key == ord("s"):
        cv2.imwrite(f"calib_{i}.jpg", frame)
        print(f"saved calib_{i}.jpg")
        i+=1
    elif key == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()