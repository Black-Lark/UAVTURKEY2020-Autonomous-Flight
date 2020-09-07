import numpy as np
import cv2
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative,Vehicle, LocationGlobal
import time
import math
vehicle = connect("udp:192.168.137.103:14550", wait_ready=True)

cap = cv2.VideoCapture(0)

def arm_and_takeoff(aTargetAltitude):
    
    """
    Arms vehicle and fly to aTargetAltitude.
    """
    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    time.sleep(2)

    print(vehicle.mode)

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.90:
            print("Reached target altitude")
            break
        time.sleep(1)


arm_and_takeoff(10)


degree = 1
east = 0
north = 0
def get_location_metres(original_location, dNorth, dEast):
    
    earth_radius = 6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)

    if type(original_location) is LocationGlobal:
        targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
    else:
        raise Exception("Invalid Location object passed")
        
    return targetlocation;

def get_distance_metres(aLocation1, aLocation2):

    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5
    
def goto(dNorth, dEast, gotoFunction=vehicle.simple_goto):
    
    currentLocation = vehicle.location.global_relative_frame
    targetLocation = get_location_metres(currentLocation, dNorth, dEast)
    targetDistance = get_distance_metres(currentLocation, targetLocation)
    gotoFunction(targetLocation)


def condition_yaw(heading, relative=False):
   
    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)

northArray = []
eastArray = [] 

while True:
    ret, frame = cap.read()
    #frame = cv2.flip(frame,-1)
    if ret == True:
        # Filter red color
        # frame = cv2.bilateralFilter(frame,9,75,75)
        frame_hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(frame_hsv, (0, 70, 50), (10, 255, 255))
        mask2 = cv2.inRange(frame_hsv, (170, 70, 50), (180, 255, 255))
        mask = mask1 + mask2
        # Edge detection
        canny_output = cv2.Canny(mask, 10, 10, 20)
        # Find the center
        pixels = np.where(canny_output == 255)
        cX = np.average(pixels[1])
        cY = np.average(pixels[0])
        
        # cX cY -> NaN
        cv2.circle(canny_output, (int(cX),int(cY)), 3, (255,255,255), thickness=10, lineType=8, shift=0)
        
        x = cX-320
        y = 240-cY
        RSquare = math.sqrt(abs(x)**2 + abs(y)**2)/10
        print(math.degrees(math.atan(y/x)))  
        degree = math.degrees(math.atan(y/x))     
        if x <0 and y<0: # 
            degree = 270 - degree
            print(degree,east,north,"3")

        elif (x<0 and y > 0): # II. Region
            degree = abs(degree-270)
            print(degree,east,north,"2")

            
        elif x>0 and y < 0:
            degree = 90-degree
            print(degree,east,north,"4")
            
        elif x>0 and y >0:
            degree = 90-degree
            print(degree,east,north,"1")
            
        east = math.sin(math.radians(degree))*RSquare
        north = math.cos(math.radians(degree))*RSquare
        
        goto(north,east)
            
        condition_yaw(degree, False)     
        cv2.imshow("contours", canny_output)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()