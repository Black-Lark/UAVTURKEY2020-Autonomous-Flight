import cv2
import numpy as np

cap = cv2.VideoCapture(0)
cX_array = np.zeros(50)
cY_array = np.zeros(50)

while True:
    _,frame = cap.read()

    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv,(0,50,50),(5,255,255))
    mask2 = cv2.inRange(hsv,(175,50,50),(255,255,255))
    mask = cv2.bitwise_or(mask1,mask2)

    cropped = cv2.bitwise_and(hsv,hsv,mask=mask)
    blur = cv2.blur(cropped,(10,10))

    # Used the gray scale image
    grayScale = cv2.cvtColor(blur,cv2.COLOR_BGR2GRAY)
    # First obtain the threshold using the grayscale image
    ret,th = cv2.threshold(grayScale,127,255,0)
    # Find all the contours in the binary image
    contours, hierarchy = cv2.findContours(th,2,1)

    cnt = contours
    big_contours = []
    max = 10
    final = frame
    for i in cnt:

        area = cv2.contourArea(i)
        M = cv2.moments(i)
        approx = cv2.approxPolyDP(i,0.01*cv2.arcLength(i,True),True)

        if len(approx) > 10:

            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
            else:
                cX = 0
                cY = 0

            cX_array = np.append(np.delete(cX_array,0),cX)
            cY_array = np.append(np.delete(cY_array, 0), cY)

            cX = int(sum(cX_array)/len(cX_array))
            cY = int(sum(cY_array) / len(cY_array))

            if(area>max):
                max = area
                big_contours = i

            cv2.circle(frame, (cX, cY), 7, (255, 255, 255), -1)
            final = cv2.drawContours(frame,big_contours,-1,(0,255,0),3)

    cv2.imshow('frame',final)
    cv2.imshow('grayscale',grayScale)
    cv2.imshow('cropped',cropped)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break