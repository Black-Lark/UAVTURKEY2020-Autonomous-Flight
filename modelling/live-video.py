import cv2
import numpy as np

cap = cv2.VideoCapture(0)

while(cap.isOpened()):
    ret, frame = cap.read()
    if ret == True:
        # Show the frame
        cv2.imshow("frame", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print('Video stream has been terminated.')
            break
    else:
        print('Video stream has been corrupted.')
        break
    
cap.release()
cv2.destroyAllWindows()