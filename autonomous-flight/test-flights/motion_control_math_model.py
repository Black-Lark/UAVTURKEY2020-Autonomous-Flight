from __future__ import print_function
import numpy as np
import cv2
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative,Vehicle, LocationGlobal, Command
from time import gmtime, strftime 
import time
import math

vehicle = connect("tcp:127.0.0.1:5762", wait_ready=True)
#vehicle = connect("/dev/serial0", wait_ready=True, baud=921000)
# OpenCV
cap = cv2.VideoCapture(0)
fourcc = cv2.VideoWriter_fourcc(*'XVID')
file_name = strftime("%Y-%m-%d_%H-%M-%S", gmtime()) + ".avi"
out = cv2.VideoWriter(file_name,fourcc, 25, (640,480))

def arm_and_takeoff(aTargetAltitude):
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    time.sleep(2)

    print(vehicle.mode)

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.90:
            print("Reached target altitude")
            break
        time.sleep(1)


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
    currentLocation=vehicle.location.global_relative_frame
    currentLocation.alt = alt
    targetLocation=get_location_metres(currentLocation, dNorth, dEast)
    targetDistance = get_distance_metres(currentLocation, targetLocation)
    gotoFunction(targetLocation)

    while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
        remainingDistance=get_distance_metres(vehicle.location.global_frame, targetLocation)
        print ("Distance to target: ", remainingDistance)
        if remainingDistance<=targetDistance*0.5: #Just below target, in case of undershoot.
            print ("Reached target")
            break
        

def distance_estimate(alt, deviation):
    alt = alt * 100
    a = 0.002
    b = -0.0129
    pixel_to_cm = a * alt + b
    return deviation * pixel_to_cm / 100


def current_yaw():
    yaw = math.degrees(vehicle.attitude.yaw)
    if yaw < 0:
        yaw = yaw + 360
    return yaw


def field_detection_tour(): 
    cmds = vehicle.commands
    cmds.clear()
    vehicle.flush()
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 5))# Dummy
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 1, 0, 0, 0, 37.0743635 ,37.276991 , 5))# 60m başlangıç
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 1, 0, 0, 0, 37.074694  ,37.2769795 , 5)) # 60m orta
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 1, 0, 0, 0, 37.0750752 ,37.2769661 , 5)) # 60m bitiş
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 1, 0, 0, 0, 37.0750752 ,37.2769661 , 5)) # Dummy 60m bitiş

    print(" Upload new commands to vehicle")
    cmds.upload()

def second_tour(lat,lon):
    cmds = vehicle.commands
    cmds.clear()
    vehicle.flush()
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 1, 0, 0, 0, lat ,lon , 5))# Su bırakma alanı
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 1, 0, 0, 0, lat ,lon , 5))# Dummy Su bırakma alanı

    # cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 1, 0, 0, 0, lat ,lon , 7))# Su bırakma
    # cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 1, 0, 0, 0, lat ,lon , 6))# 
    # cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 1, 0, 0, 0, lat ,lon , 5))# 
    # cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 1, 0, 0, 0, lat ,lon , 4))# 
    # cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 1, 0, 0, 0, lat ,lon , 3))# 
    # cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 1, 0, 0, 0, lat ,lon , 2))# 
    # cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 1, 0, 0, 0, lat ,lon , 1))# 
    # cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 1, 0, 0, 0, lat ,lon , 1))# 1m dummy

    cmds.upload()

def second_tour_part_two(center_lat, center_lon):
    cmds = vehicle.commands
    cmds.clear()
    vehicle.flush() 
    #cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 5)) # Delete in real fligth
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, center_lat , center_lon, 5))# Su bırakma
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, center_lat , center_lon, 4))# Su bırakma
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, center_lat , center_lon, 3))# Su bırakma
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, center_lat , center_lon, 2))# Su bırakma
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, center_lat , center_lon, 1.75))# Su bırakma
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, center_lat , center_lon, 1))# Su bırakma
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, center_lat , center_lon, 1))# 1m dummy bırakma

    cmds.upload()

arm_and_takeoff(5)
# Calling field_detection tour
field_detection_tour()
vehicle.commands.next = 0
nextwaypoint = 0

while vehicle.mode != "AUTO":
    vehicle.mode = VehicleMode("AUTO")
    print("waiting auto mode")
lat = 0
lon = 0
frame_pos = []
r_square = []
while vehicle.commands.next <=3:
    nextwaypoint=vehicle.commands.next
    if (vehicle.commands.next == 2 or  vehicle.commands.next == 3):
        ret, frame = cap.read()

        if ret == True:
            # Filter red color
            #frame = cv2.bilateralFilter(frame,9,75,75)
            frame_hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
            mask1 = cv2.inRange(frame_hsv, (0, 70, 50), (10, 255, 255))
            mask2 = cv2.inRange(frame_hsv, (160, 70, 50), (180, 255, 255))
            mask = mask1 + mask2
            white_pixels = np.where(mask==255)
            cX = np.average(white_pixels[1])
            cY = np.average(white_pixels[0])
            
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
                        #Show the frame
                        cv2.imshow("intersection", intersection)
                        intersection_cX= np.average(intersection_length[1])
                        intersection_cY= np.average(intersection_length[0])
                        x = intersection_cX-320
                        y = 240-intersection_cY
                        RSquare = math.sqrt((x)*(x) + (y)*(y))
                        lat = vehicle.location.global_relative_frame.lat
                        lon = vehicle.location.global_relative_frame.lon
                        frame_pos.append([lat,lon])
                        r_square.append(RSquare)
                        # Getting the nearest location
            # Show the frame        
            cv2.imshow("frame", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print('Video stream has been terminated.')
                break



while vehicle.mode != "GUIDED":
    vehicle.mode = VehicleMode("GUIDED")
    print("waiting guided mode")

cap.release()
out.release()
cv2.destroyAllWindows()

cap = cv2.VideoCapture(0)
MinPosition = r_square.index(min(r_square))
lat,lon = frame_pos[MinPosition]   
print(lat,lon)
second_tour(lat,lon)
vehicle.commands.next = 0
nextwaypoint = 0

while vehicle.mode != "AUTO":
    vehicle.mode = VehicleMode("AUTO")
    print("waiting auto mode")


while vehicle.commands.next <=1:
    nextwaypoint=vehicle.commands.next
    print("Next Wp is ", vehicle.commands.next)


while vehicle.mode != "GUIDED":
    vehicle.mode = VehicleMode("GUIDED")
    print("waiting for guided line 237")

while True: 
    ret, frame = cap.read()
    out.write(frame)
    if ret == True:
        # Filter red color
        cv2.imshow("frame",frame)
        #frame = cv2.bilateralFilter(frame,9,75,75)
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
            cv2.circle(img, (int(cX),int(cY)), 85, (255,255,255), thickness=-1, lineType=8, shift=0) # 85 değişecek
            intersection = cv2.bitwise_and(img,mask)
            intersection_length = np.where(intersection==255)

            # Grande noise elimination
            if len(intersection_length[0]) > 5000:
                # Show the frame
                intersection_cX= np.average(intersection_length[1])
                intersection_cY= np.average(intersection_length[0])
                cv2.imshow("intersection", intersection)
                # Set 
                x = intersection_cX-320
                y = 240-intersection_cY
                # deviation = math.sqrt((x)*(x) + (y)*(y))
                #rangefinder_alt = vehicle.location.global_relative_frame.alt
                rangefinder_alt = vehicle.rangefinder.distance
                # Get deviation in meters at x-axis
                x = distance_estimate(rangefinder_alt, x)
                # Get deviation in meters at y-axis
                y = distance_estimate(rangefinder_alt, y)
                
                # Current yaw in degrees
                alpha = current_yaw()
                # Deviation vector degree in camera frame
                if x == 0:
                    theta = 90
                else:
                    theta = math.degrees(math.atan(y/x))
                # Deviation vector degree in global frame
                beta = theta - alpha
                # Deviation vector magnitude
                d = math.sqrt(x*x + y*y)
                # Deviation vector components in global frame
                north = d * math.sin(math.radians(beta))
                east = d * math.cos(math.radians(beta))

                # Correction for 2. and 3.
                if (x < 0 and y < 0) or (x < 0 and y > 0):
                    north = north * -1
                    east = east * -1

                print("theta: ", theta)
                print(east, ", ", north)

                # Go to the location
                goto(north*2, east*2, vehicle.rangefinder.distance)
                break

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
cap.release()
out.release()
cv2.destroyAllWindows()

center_lat = vehicle.location.global_relative_frame.lat
center_lon = vehicle.location.global_relative_frame.lon

print("center lat", center_lat, "center long", center_lon)
second_tour_part_two(center_lat,center_lon)
vehicle.commands.next=0
nextwaypoint=0
while vehicle.mode != "AUTO":
    vehicle.mode = VehicleMode("AUTO")
    print("waiting for last auto line 340")


while vehicle.location.global_relative_frame.alt >= 1.5:
    
    nextwaypoint=vehicle.commands.next

vehicle.mode = VehicleMode("LOITER")