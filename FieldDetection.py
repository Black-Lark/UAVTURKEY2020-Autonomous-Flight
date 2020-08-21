import cv2
import numpy as np

cap = cv2.VideoCapture(0)

while(cap.isOpened()):
    ret, frame = cap.read()

    if ret == True:
        # Filter red color
        frame_hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(frame_hsv, (0, 70, 50), (10, 255, 255))
        mask2 = cv2.inRange(frame_hsv, (170, 70, 50), (180, 255, 255))
        mask = mask1 + mask2

        # cv2.imshow("frame", frame)
        # cv2.imshow("mask", mask)

        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

        canny_output = cv2.Canny(mask, 10, 10 * 2)
        contours, _ = cv2.findContours(canny_output, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Get the moments
        mu = [None]*len(contours)
        for i in range(len(contours)):
            mu[i] = cv2.moments(contours[i])
        # Get the mass centers
        mc = [None]*len(contours)
        for i in range(len(contours)):
            # add 1e-5 to avoid division by zero
            mc[i] = (mu[i]['m10'] / (mu[i]['m00'] + 1e-5), mu[i]['m01'] / (mu[i]['m00'] + 1e-5))
        # Draw contours
        
        drawing = np.zeros((canny_output.shape[0], canny_output.shape[1], 3), dtype=np.uint8)
        
        for i in range(len(contours)):
            cv2.drawContours(drawing, contours, i, (0,255,255), 2)
            cv2.circle(drawing, (int(mc[i][0]), int(mc[i][1])), 4, (0,255,255), -1)
        
        cv2.imshow('Contours', drawing)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            print('Video stream has been terminated.')
            break

    else:
        print('Video stream has been corrupted.')
        break

cap.release()
cv2.destroyAllWindows()