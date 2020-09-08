from __future__ import print_function
import numpy as np
import cv2
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative,Vehicle, LocationGlobal, Command
import time
import math

#vehicle = connect("tcp:127.0.0.1:5762", wait_ready=True)
vehicle = connect("/dev/serial0", wait_ready=True, baud=921000)

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
        
    return targetlocation

def get_distance_metres(aLocation1, aLocation2):

    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5
    
def goto(dNorth, dEast, alt, gotoFunction=vehicle.simple_goto):
    
    currentLocation = vehicle.location.global_relative_frame
    currentLocation.alt = alt
    targetLocation = get_location_metres(currentLocation, dNorth, dEast)
    targetDistance = get_distance_metres(currentLocation, targetLocation)
    gotoFunction(targetLocation)

    while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
        remainingDistance=get_distance_metres(vehicle.location.global_frame, targetLocation)
        #print ("Distance to target: ", remainingDistance)
        if remainingDistance<=0.3: #Just below target, in case of undershoot.
            #print ("Reached target")
            time.sleep(0.5)
            break

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

def first_tour(): 
    cmds = vehicle.commands
    cmds.clear()
    vehicle.flush()
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 5))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 2, 0, 0, 0, 38.6942368 ,35.4605909 , 5))#1 Su alma alanı
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 2, 0, 0, 0, 38.6942368 ,35.4605909 , 5))#1 Su alma alanı

    print(" Upload new commands to vehicle")
    cmds.upload()
def arm_and_takeoff(aTargetAltitude):
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:      
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)      
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)

arm_and_takeoff(5)
cap = cv2.VideoCapture(0)
degree = 1
east = 0
north = 0
counter = 0
frame_pos = []
first_tour()
vehicle.mode = VehicleMode("AUTO")
vehicle.commands.next=0

while vehicle.commands.next <=1:
    
    nextwaypoint=vehicle.commands.next

vehicle.mode = VehicleMode("GUIDED")
while True: 
    ret, frame = cap.read()
    frame = cv2.flip(frame,1)
    if ret == True:
        # Filter red color
        frame = cv2.bilateralFilter(frame,9,75,75)
        frame_hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(frame_hsv, (0, 70, 50), (10, 255, 255))
        mask2 = cv2.inRange(frame_hsv, (165, 70, 50), (180, 255, 255))
        mask = mask1 + mask2
        white_pixels = np.where(mask==255)
        cX = np.average(white_pixels[1])
        cY = np.average(white_pixels[0])
        
        # Small noise elimination
        if len(white_pixels[0]) > 5000:
            # Object location detection
            img = np.zeros((480,640,1),np.uint8)    
            cv2.circle(img, (int(cX),int(cY)), 85, (255,255,255), thickness=-1, lineType=8, shift=0)
            intersection = cv2.bitwise_and(img,mask)
            intersection_length = np.where(intersection==255)

            #print(len(intersection_length[0]))
            # Grande noise elimination
            if len(intersection_length[0]) > 5000:
                # Show the frame
                intersection_cX= np.average(intersection_length[1])
                intersection_cY= np.average(intersection_length[0])
                #cv2.imshow("mask", mask)
                #cv2.imshow("black", img)
                cv2.imshow("intersection", intersection)

                x = intersection_cX-320
                y = 240-intersection_cY
                RSquare = math.sqrt(abs(x)*abs(x) + abs(y)*abs(y))
                degree = math.degrees(math.atan(y/x))
                
                if x <0 and y<0: # 
                    degree = 270 - degree

                elif (x<0 and y > 0): # II. Region
                    degree = abs(degree-270)

                elif x>0 and y < 0:
                    degree = 90-degree

                elif x>0 and y >0:
                    degree = 90-degree
<<<<<<< Updated upstream
              
                counter = counter + 1 
                if counter % 3 == 0:    
                    east = math.sin(math.radians(degree))/5
                    north = math.cos(math.radians(degree))/5
=======

                east = math.sin(math.radians(degree))*RSquare/50
                north = math.cos(math.radians(degree))*RSquare/50
>>>>>>> Stashed changes

                if RSquare > 40:
                    condition_yaw(degree)
                    time.sleep(1)
                    print("Yaw is set")
                    goto(north,east,vehicle.location.global_relative_frame.alt) # field = True #Field is centered. Ready to drop the water
                    print("point is reached")

                elif RSquare < 40:
                    while vehicle.location.global_relative_frame.alt>50:
                        vehicle.location.global_relative_frame.alt-0.35
                        print("landing")
       

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

