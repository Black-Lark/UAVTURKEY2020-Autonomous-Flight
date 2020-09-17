import cv2
import numpy as np

cap = cv2.VideoCapture(0)
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))

while(cap.isOpened()):
    ret, frame = cap.read()
    if ret == True:
        # Filter red color
        frame = cv2.bilateralFilter(frame,9,75,75)
        frame_hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(frame_hsv, (0, 70, 50), (10, 255, 255))
        mask2 = cv2.inRange(frame_hsv, (170, 70, 50), (180, 255, 255))
        mask = mask1 + mask2
        white_pixels = np.where(mask==255)
        cX = np.average(white_pixels[1])
        cY = np.average(white_pixels[0])
        cv2.imshow("mask",mask)
        # Small noise elimination
        if len(white_pixels[0]) > 5000:
            # Object location detection
            if (220 < cY < 350):
                img = np.zeros((480,640,1),np.uint8)    
                cv2.circle(img, (int(cX),int(cY)), 85, (255,255,255), thickness=-1, lineType=8, shift=0)
                intersection = cv2.bitwise_and(img,mask)
                intersection_length = np.where(intersection==255)

                print(len(intersection_length[0]))
                # Grande noise elimination
                if len(intersection_length[0]) > 5000:
                    # Show the frame
                    cv2.imshow("mask", mask)
                    cv2.imshow("black", img)
                    cv2.imshow("intersection", intersection)
                    #cv2.imwrite("test.png",intersection)
                    
        # Show the frame        
        cv2.imshow("frame", frame)
        if cv2.waitKey(500) & 0xFF == ord('q'):
            print('Video stream has been terminated.')
            break

    else:
        print('Video stream has been corrupted.')
        break

cap.release()
cv2.destroyAllWindows()