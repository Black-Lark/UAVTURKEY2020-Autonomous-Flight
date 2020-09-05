import cv2
import numpy as np

cap = cv2.VideoCapture(0)
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
print(frame_height, " " ,frame_width)
while(cap.isOpened()):
    ret, frame = cap.read()
    if ret == True:
        # Show the frame
        # out.write(frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print('Video stream has been terminated.')
            break
    else:
        print('Video stream has been corrupted.')
        break
out.release()
cap.release()
cv2.destroyAllWindows()