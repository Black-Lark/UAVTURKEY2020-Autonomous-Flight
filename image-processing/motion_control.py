while True:

    print(x,y)
    _, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, (0,50,50), (5,255,255))
    mask2 = cv2.inRange(hsv, (175,50,50), (255,255,255))
    mask = cv2.bitwise_or(mask1, mask2 )
    croped = cv2.bitwise_and(hsv, hsv, mask=mask)
    blur = cv2.blur(croped,(10,10))
    gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)

    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1.5, 150, minRadius=10, maxRadius=300)
    
    if circles is not None:
        # convert the (x, y) coordinates and radius of the circles to integers
        circles = np.round(circles[0, :]).astype("int")
        # loop over the (x, y) coordinates and radius of the circles
        for (x, y, r) in circles:
            # draw the circle in the output image, then draw a rectangle
            # corresponding to the center of the circle
            cv2.circle(frame, (x, y), r, (0, 255, 0), 4)
            cv2.circle(frame, (x, y), 1, (0, 0, 255), 2)
            cv2.line(frame,(256,256),(x,y),(0,0,255),2)
    y1 = y-256
    x1 = x-256        
    m = (y/x)
    (yc-y1) = m*(xc-x1)
    send_ned_velocity((x-256)/20, (y-256)/20, 0, 1)
        
    # show the output image
    #cv2.imshow("gray", gray)
    cv2.imshow("frame", frame)
    #cv2.imshow("croped", croped)
    
    #cv2.imshow("hsv", frame1)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()