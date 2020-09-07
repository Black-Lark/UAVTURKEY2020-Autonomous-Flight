import numpy as np
import cv2
#from picamera import PiCamera
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative,Vehicle, LocationGlobal, Command
import time
from time import gmtime, strftime

# vehicle = connect("/dev/serial0", wait_ready=True, baud=921000)
vehicle = connect("tcp:127.0.0.1:5762", wait_ready=True)
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

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.90:
            print("Reached target altitude")
            break
        time.sleep(1)

def first_tour(): 
    cmds = vehicle.commands
    cmds.clear()
    vehicle.flush()
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 5))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 2, 0, 0, 0, 38.6941981 ,35.4605775 , 5))#1 Su alma alanı
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 38.6943938, 35.4600330 , 5))#2
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 38.694307, 35.4599431 , 5))#3
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 38.6940652, 35.4605828 , 5))#4
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 38.6941650 ,35.4606727 , 5))#5 Starting point
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 38.6941650 ,35.4606727 , 5))#5 Starting point Dummy
 

    print(" Upload new commands to vehicle")
    cmds.upload()

def second_tour(lat,lon):
    cmds = vehicle.commands
    cmds.clear()
    vehicle.flush() 
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 5))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 1, 0, 0, 0, 38.6941981 ,35.4605775 , 5))#1 Su alma alanı
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 3, 0, 0, 0, 38.6941981 ,35.4605775 , 1))#1 Su alma alanı
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 1, 0, 0, 0, 38.6941981 ,35.4605775 , 5))#1 Su alma alanı
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 1, 0, 0, 0, lat ,lon , 5))#2 Su bırakma alanı
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 3, 0, 0, 0, lat ,lon , 1))#2 Su bırakma alanı
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 38.6943938, 35.4600330 , 5))#2
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 38.694307, 35.4599431 , 5))#3
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 38.6940652, 35.4605828 , 5))#4
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 38.6941650 ,35.4606727 , 5))#5 Starting point
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 38.6941650 ,35.4606727 , 5))#5 Starting point Dummy

    cmds.upload()

arm_and_takeoff(5)
first_tour()
vehicle.mode = VehicleMode("AUTO")
vehicle.commands.next=0

# Detect coordinates of the field
lat = 0
lon = 0
r_square = []
pos = []

while vehicle.commands.next <=5:
    
    nextwaypoint=vehicle.commands.next
    if vehicle.commands.next == 2:
        ret, frame = cap.read()
        
        if ret == True:
            # Save the video
            out.write(frame)
            # Filter red color
            #frame = cv2.bilateralFilter(frame,9,75,75)
            frame_hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
            mask1 = cv2.inRange(frame_hsv, (0, 70, 50), (10, 255, 255))
            mask2 = cv2.inRange(frame_hsv, (165, 70, 50), (180, 255, 255))
            mask = mask1 + mask2
            white_pixels = np.where(mask==255)
            cX = np.average(white_pixels[1])
            cY = np.average(white_pixels[0])
            rX = cX - 320
            rY = 240 - cY
            r = rX*rX + rY*rY # need to be changed

            # Small noise elimination
            if len(white_pixels[0]) > 4000:

                # Object location detection
                if (220 < cY < 350):
                    img = np.zeros((480,640,1),np.uint8)    
                    cv2.circle(img, (int(cX),int(cY)), 85, (255,255,255), thickness=-1, lineType=8, shift=0)
                    intersection = cv2.bitwise_and(img,mask)
                    intersection_length = np.where(intersection==255)

                    print(len(intersection_length[0]))
                    # Grande noise elimination
                    if len(intersection_length[0]) > 4000:
                        # Show the frame
                        print("Field Detected")
                        lat = vehicle.location.global_relative_frame.lat
                        lon = vehicle.location.global_relative_frame.lon
                        pos.append([lat,lon])
                        r_square.append(r)
            
            # Show the frame        
            cv2.imshow("frame", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print('Video stream has been terminated.')
                break

        else:
            print('Video stream has been corrupted.')
            break

cap.release()
cv2.destroyAllWindows()
print("length of r square =", len(r_square))
if r_square is not None:
    index = r_square.index(min(r_square))
    finalpos = pos[index]
    print(finalpos[0], finalpos[1])
    second_tour(finalpos[0],finalpos[1])
    vehicle.commands.next=0
    nextwaypoint=0

    while vehicle.commands.next <=9:
        nextwaypoint=vehicle.commands.next

print("Out")
vehicle.mode = VehicleMode("LAND")
