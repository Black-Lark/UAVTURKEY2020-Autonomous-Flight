import cv2
import numpy as np

cap = cv2.VideoCapture(0)
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))

while(cap.isOpened()):
    ret, frame = cap.read()
    if ret == True:
        # Filter red color
        # frame = cv2.bilateralFilter(frame,9,75,75)
        frame_hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(frame_hsv, (0, 70, 50), (10, 255, 255))
        mask2 = cv2.inRange(frame_hsv, (170, 70, 50), (180, 255, 255))
        mask = mask1 + mask2
        white_pixels = np.where(mask==255)
        print(len(white_pixels[1]))
        
        # Edge detection
        canny_output = cv2.Canny(mask, 10, 10, 20)
        # Find the center
        pixels = np.where(canny_output == 255)
        pixel_max = max(pixels[0])
        pixel_min = min(pixels[0])
       
        pixel_max_x = max(pixels[1])
        pixel_min_x = min(pixels[1])
       
        Diameter = abs(pixel_max-pixel_min)
        Diameter_x = abs(pixel_max_x - pixel_min_x)
        #print(Diameter, Diameter_x)
        cX = np.average(pixels[1])
        cY = np.average(pixels[0])

        # cX cY -> NaN
        cv2.circle(canny_output, (int(cX),int(cY)), 3, (255,255,255), thickness=10, lineType=8, shift=0)
        # Show the frame
        cv2.imshow("contours", canny_output)
        cv2.imshow("mask", mask)
        #cv2.imshow("frame",frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print('Video stream has been terminated.')
            break

    else:
        print('Video stream has been corrupted.')
        break

cap.release()
cv2.destroyAllWindows()