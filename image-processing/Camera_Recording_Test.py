import cv2
import numpy as np
from time import gmtime, strftime
from picamera import PiCamera
from time import sleep
dronecamera = PiCamera()
dronecamera.start_recording('C:\Users\thosl\Documents\GitHub/video_test.h264')
cap = cv2.VideoCapture(0)
#fourcc = cv2.VideoWriter_fourcc(*'XVID')
#file_name = strftime("%Y-%m-%d_%H-%M-%S", gmtime()) + ".avi"
#out = cv2.VideoWriter(file_name,fourcc, 25, (640,480))
#print(file_name)
sleep(10)
dronecamera.stop_recording()
#out = cv2.VideoWriter(file_name,fourcc, 30, (640,480))
while(cap.isOpened()):
    ret, frame = cap.read()
    if ret == True:
        frame_hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(frame_hsv, (0, 70, 50), (10, 255, 255))
        mask2 = cv2.inRange(frame_hsv, (165, 70, 50), (180, 255, 255))
        mask = mask1 + mask2
        # Show the frame
        #out.write(mask)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print('Video stream has been terminated.')
            break
    else:
        print('Video stream has been corrupted.')
        break
out.release()
cap.release()
cv2.destroyAllWindows()