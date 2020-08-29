from __future__ import print_function
import numpy as np
import cv2
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative,Vehicle, LocationGlobal, Command
import time
import math

vehicle = connect("tcp:127.0.0.1:5762", wait_ready=True)

def first_tour(): 
    cmds = vehicle.commands
    print("Clear any existing commands")
    cmds.clear() 
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 37.0747478 ,37.2706568 , 10)) #1
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 37.0747927 ,37.2709626 , 10)) #2
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 37.0749917 ,37.2711074 , 10)) #3
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 37.0751308 ,37.2709277 , 10)) #4
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 37.0750816 ,37.2701043 , 10)) #5
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 37.0749917 ,37.2699514 , 10)) #6
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 37.0747670 ,37.2698817 , 10)) #7
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 37.0747526 ,37.2702458 , 10)) #8
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 37.0747526 ,37.2702458 , 10)) #8/
    print(" Upload new commands to vehicle")
    cmds.upload()

def second_tour(lat,lon):

    cmds = vehicle.commands
    print("Clear any existing commands")
    cmds.clear()
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 37.0747478 ,37.2706568 , 10)) #1
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 37.0747927 ,37.2709626 , 10)) #2
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 37.0749917 ,37.2711074 , 0.25)) #Location of taking water
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 37.0751308 ,37.2709277 , 10)) #4
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, lat , lon , 10)) #Location of water drop
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 37.0750816 ,37.2701043 , 10)) #5
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 37.0749917 ,37.2699514 , 10)) #6
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 37.0747670 ,37.2698817 , 10)) #7
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 37.0747526 ,37.2702458 , 10)) #8
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 37.0747526 ,37.2702458 , 10)) #8/
    print(" Upload new commands to vehicle")
    cmds.upload()

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

arm_and_takeoff(10)
first_tour()

print("Starting mission")
# Reset mission set to first (0) waypoint
vehicle.commands.next=0

# Set mode to AUTO to start mission
vehicle.mode = VehicleMode("AUTO")
cap = cv2.VideoCapture(0)
lat = 0
lon = 0
while vehicle.commands.next <= 8:
    nextwaypoint=vehicle.commands.next
    if vehicle.commands.next==5:
        ret, frame = cap.read()
        frame = cv2.flip(frame,1)
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
            cv2.imshow("canny_output", canny_output)
            x = cX-320
            y = 240-cY
            if (x and y) > 10:
                lat = vehicle.location.global_relative_frame.lat
                lon = vehicle.location.global_relative_frame.lon
                print(lat,lon)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

print("Starting mission")
# Reset mission set to first (0) waypoint
cap.release()
cv2.destroyAllWindows()
time.sleep(2)
second_tour(lat,lon)
nextwaypoint=0
vehicle.commands.next=0

cap = cv2.VideoCapture(0)
degree = 1
east = 0
north = 0
field = False
while vehicle.commands.next<=9:
    nextwaypoint=vehicle.commands.next
    if nextwaypoint == 4:
        vehicle.mode = VehicleMode("GUIDED")
        time.sleep(6)
        print("Ready to take water")
        vehicle.mode = VehicleMode("AUTO")

    if field == False and vehicle.commands.next==7:
        vehicle.mode = VehicleMode("GUIDED")  
        ret, frame = cap.read()
        frame = cv2.flip(frame,1)
        if ret == True:
            
            # Filter red color
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
            degree = math.degrees(math.atan(y/x))

            if x <0 and y<0: # 
                degree = 270 - degree

            elif (x<0 and y > 0): # II. Region
                degree = abs(degree-270)

            elif x>0 and y < 0:
                degree = 90-degree

            elif x>0 and y >0:
                degree = 90-degree

            east = math.sin(math.radians(degree))*RSquare
            north = math.cos(math.radians(degree))*RSquare
            if RSquare < 3:
                goto(north,east)
                print("I am in")
                field = True
                #field = True #Field is centered. Ready to drop the water
            cv2.imshow("canny_output", canny_output)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

print('Return to launch')
vehicle.mode = VehicleMode("RTL")

print("Close vehicle object")
vehicle.close()
