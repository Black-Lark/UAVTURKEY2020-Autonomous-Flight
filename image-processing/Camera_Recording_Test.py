import cv2
import numpy as np
from time import gmtime, strftime

cap = cv2.VideoCapture(0)
fourcc = cv2.VideoWriter_fourcc(*'XVID')
file_name = strftime("%Y-%m-%d_%H-%M-%S", gmtime()) + ".avi"
out = cv2.VideoWriter(file_name,fourcc, 25, (640,480))
print(file_name)
counter = 0
while(cap.isOpened()):
    ret, frame = cap.read()
    if ret == True:
        # frame_hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        # mask1 = cv2.inRange(frame_hsv, (0, 70, 50), (10, 255, 255))
        # mask2 = cv2.inRange(frame_hsv, (165, 70, 50), (180, 255, 255))
        # mask = mask1 + mask2
        # Show the frame
        counter = counter + 1
        out.write(frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print('Video stream has been terminated.')
            break
    else:
        print('Video stream has been corrupted.')
        break
print(counter)
cap.release()
out.release()
cv2.destroyAllWindows()