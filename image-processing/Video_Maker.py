import cv2
import numpy as np
from time import gmtime, strftime

cap = cv2.VideoCapture(0)
#file_name = strftime("%Y-%m-%d_%H-%M-%S", gmtime()) + ".avi"
#fourcc = cv2.VideoWriter_fourcc(*'MJPG')
#frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
#frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
#out = cv2.VideoWriter(file_name,fourcc, 30, (240,360))
#print(file_name)
while(cap.isOpened()):
    ret, frame = cap.read()
    if ret == True:
 
        counter = counter +1
        cv2.imwrite(str(counter) +".png",frame)
        # Show the frame
        out.write(frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print('Video stream has been terminated.')
            break
    else:
        print('Video stream has been corrupted.')
        break
cap.release()
cv2.destroyAllWindows()