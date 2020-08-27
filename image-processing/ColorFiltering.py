import cv2
import numpy as np

cap = cv2.VideoCapture(0)

def trackingCall(x):
    print(x)

# Create new window
cv2.namedWindow("Tracking")
cv2.createTrackbar("mask1_H", "Tracking", 0, 255, trackingCall)
cv2.createTrackbar("mask1_S", "Tracking", 0, 255, trackingCall)
cv2.createTrackbar("mask1_V", "Tracking", 0, 255, trackingCall)
cv2.createTrackbar("mask1_H2", "Tracking", 255, 255, trackingCall)
cv2.createTrackbar("mask1_S2", "Tracking", 255, 255, trackingCall)
cv2.createTrackbar("mask1_V2", "Tracking", 255, 255, trackingCall)

cv2.createTrackbar("mask2_H", "Tracking", 0, 255, trackingCall)
cv2.createTrackbar("mask2_S", "Tracking", 0, 255, trackingCall)
cv2.createTrackbar("mask2_V", "Tracking", 0, 255, trackingCall)
cv2.createTrackbar("mask2_H2", "Tracking", 255, 255, trackingCall)
cv2.createTrackbar("mask2_S2", "Tracking", 255, 255, trackingCall)
cv2.createTrackbar("mask2_V2", "Tracking", 255, 255, trackingCall)

while(cap.isOpened()):
    ret, frame = cap.read()

    if ret == True:

        m1h = cv2.getTrackbarPos("mask1_H", "Tracking")
        m1s = cv2.getTrackbarPos("mask1_S", "Tracking")
        m1v = cv2.getTrackbarPos("mask1_V", "Tracking")
        m1h2 = cv2.getTrackbarPos("mask1_H2", "Tracking")
        m1s2 = cv2.getTrackbarPos("mask1_S2", "Tracking")
        m1v2 = cv2.getTrackbarPos("mask1_V2", "Tracking")

        m2h = cv2.getTrackbarPos("mask2_H", "Tracking")
        m2s = cv2.getTrackbarPos("mask2_S", "Tracking")
        m2v = cv2.getTrackbarPos("mask2_V", "Tracking")
        m2h2 = cv2.getTrackbarPos("mask2_H2", "Tracking")
        m2s2 = cv2.getTrackbarPos("mask2_S2", "Tracking")
        m2v2 = cv2.getTrackbarPos("mask2_V2", "Tracking")

        # Filter red color
        frame_hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(frame_hsv, (m1h, m1s, m1v), (m1h2, m1s2, m1v2))
        mask2 = cv2.inRange(frame_hsv, (m2h, m2s, m2v), (m2h2, m2s2, m2v2))
        mask = mask1 + mask2

        cv2.imshow("frame", frame)
        cv2.imshow("mask", mask)


        if cv2.waitKey(1) & 0xFF == ord('q'):
            print('Video stream has been terminated.')
            break

    else:
        print('Video stream has been corrupted.')
        break

cap.release()
cv2.destroyAllWindows()