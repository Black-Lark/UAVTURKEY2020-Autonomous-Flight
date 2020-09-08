import cv2
import numpy as np
from time import gmtime, strftime

cap = cv2.VideoCapture(0)
fourcc = cv2.VideoWriter_fourcc(*'XVID')
file_name = strftime("%Y-%m-%d_%H-%M-%S", gmtime()) + ".avi"
out = cv2.VideoWriter(file_name,fourcc, 25, (640,480))
print(file_name)

out = cv2.VideoWriter(file_name,fourcc, 30, (640,480))
while(cap.isOpened()):
    ret, frame = cap.read()
    if ret == True:
        # Show the frame
        out.write(frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print('Video stream has been terminated.')
            break
    else:
        print('Video stream has been corrupted.')
        break
out.release()
cap.release()
cv2.destroyAllWindows()