import numpy as np
import cv2
import time
import math

def distance_estimate(alt, deviation):
    alt = alt * 100
    a = 0.002
    b = -0.0129
    pixel_to_cm = a * alt + b
    return deviation * pixel_to_cm / 100


cap = cv2.VideoCapture(0)
distance = 0.7

while True: 
    ret, frame = cap.read()
    if ret == True:
        # Flip the image
        frame = cv2.flip(frame, 1)
        # cv2.imshow("frame",frame)
        # Mask green color
        frame_hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        low_green = np.array([25, 52, 72])
        high_green = np.array([102, 255, 255])
        mask = cv2.inRange(frame_hsv, low_green, high_green)
        cv2.imshow("mask",mask)

        frame_bgr = cv2.cvtColor(frame_hsv,cv2.COLOR_HSV2BGR)
        frame_green = cv2.bitwise_and(frame_bgr, frame_bgr, mask=mask)

        # Show final mask
        # cv2.imshow("green mask", frame_green)
        # Find the center
        white_pixels = np.where(mask==255)
        cX = np.average(white_pixels[1])
        cY = np.average(white_pixels[0])
       
        # Small noise elimination
        if len(white_pixels[0]) > 1000:
            # Create a black image
            black_img = np.zeros((480,640,1),np.uint8)    
            # Draw a circle 
            cv2.circle(black_img, (int(cX),int(cY)), 85, (255,255,255), thickness=-1, lineType=8, shift=0)
            intersection = cv2.bitwise_and(black_img, mask)
            cv2.imshow("intersection",intersection)
            intersection_length = np.where(intersection==255)

            # Noise elimination II
            if len(intersection_length[0]) > 1000:
                intersection_cX= np.average(intersection_length[1])
                intersection_cY= np.average(intersection_length[0])
                
                # Calculate deviations 
                x = intersection_cX-320
                y = 240-intersection_cY
                deviation = math.sqrt((x)*(x) + (y)*(y))
                
                # Draw the center
                cv2.circle(frame, (int(intersection_cX),int(intersection_cY)), 10, (255,255,255), thickness=-1, lineType=8, shift=0)
                # Draw deviation line
                cv2.line(frame, (320, 240), (int(intersection_cX),int(intersection_cY)), (255, 255, 255), 2)
                cv2.imshow("frame updated", frame)
                
                # Get deviation in meters at x-axis
                x = distance_estimate(distance, x)
                # Get deviation in meters at y-axis
                y = distance_estimate(distance, y)
                print("(" + str(x) + ", " + str(y) + ")")

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
cap.release()
cv2.destroyAllWindows()