import numpy as np
import cv2
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative,Vehicle, LocationGlobal, Command
import time
from time import gmtime, strftime
import math
#vehicle = connect("/dev/serial0", wait_ready=True, baud=921000)
vehicle = connect("tcp:127.0.0.1:5762", wait_ready=True)

# Recording
cap = cv2.VideoCapture(0)
fourcc = cv2.VideoWriter_fourcc(*'XVID')
file_name = strftime("%Y-%m-%d_%H-%M-%S", gmtime()) + ".avi"
out = cv2.VideoWriter(file_name,fourcc, 30, (640,480))
print(file_name)

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
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.90:
            print("Reached target altitude")
            break
        time.sleep(1)


def field_detection_tour():
    cmds = vehicle.commands
    cmds.clear()
    vehicle.flush()
    
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 8)) # Dummy wp
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 37.0744883 , 37.2768346, 8))#27
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 37.0743075 , 37.2768393, 8))#28
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 37.0742171 , 37.276838, 8))#29
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 37.0741662 , 37.2768541, 8))#30
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 37.0741411 , 37.2769164, 8))#31
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 37.0741518 , 37.2769694, 8))#32
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 37.0741769 , 37.2769888, 8))#33
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 37.0742567 , 37.2769922, 8))#34
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 37.0742954 , 37.276994, 8))#8
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 37.0743635 , 37.276991, 8))# 60 meters starting point
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 37.0749091 , 37.2769729, 8))# 60 meters finishing point
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 37.0750752 , 37.2769661, 8))# 19 (Direk)
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 37.0751333 , 37.2769664, 8))# 20 
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 37.0751282 , 37.2768145, 8))# 21 
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 37.0748864 , 37.2768279, 8))# 23
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 37.0746205 , 37.2768293, 8))# 25
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 37.0746205 , 37.2768293, 8))# 25 Dummy

    cmds.upload()

def second_tour(lat,lon):
    cmds = vehicle.commands
    cmds.clear()
    vehicle.flush() 
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 37.0744883 , 37.2768346, 8))#27
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 37.0743075 , 37.2768393, 8))#28
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 37.0742171 , 37.276838, 8))#29
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 37.0741662 , 37.2768541, 8))#30
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 37.0741411 , 37.2769164, 8))#31
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 37.0741898 , 37.2769882, 8))#32
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 2, 0, 0, 0, 37.0743067 , 37.2769248, 8))#Su alma   
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 37.0743067 , 37.2769248, 6))#Su alma   
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 37.0743067 , 37.2769248, 4))#Su alma   
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 37.0743067 , 37.2769248, 2))#Su alma   
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 37.0743067 , 37.2769248, 1))#Su alma
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 37.0743067 , 37.2769248, 0.6))#Su alma
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 37.0743067 , 37.2769248, 0.4))#Su alma

    cmds.upload()
    cmds.wait_ready()


# Arm and take off
arm_and_takeoff(8)
# Calling field_detection tour
field_detection_tour()
vehicle.commands.next = 0
nextwaypoint = 0
vehicle.mode = VehicleMode("AUTO")
lat = 0
lon = 0
frame_pos = []
r_square = []
while vehicle.commands.next <=16:
    nextwaypoint=vehicle.commands.next
    if (vehicle.commands.next == 10 or  vehicle.commands.next == 11):
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

cap.release()
out.release()
cv2.destroyAllWindows()
cap = cv2.VideoCapture(0)
MinPosition = r_square.index(min(r_square))
lat,lon = frame_pos[MinPosition]   
print(lat,lon)
second_tour(lat,lon)
vehicle.commands.next=0
nextwaypoint=0

vehicle.mode = VehicleMode("AUTO")

while vehicle.location.global_relative_frame.alt > 0.5: #vehicle.rangefinder.distance

    nextwaypoint=vehicle.commands.next

print("out")
vehicle.mode = VehicleMode("LOITER")


