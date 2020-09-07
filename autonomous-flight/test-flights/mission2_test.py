import numpy as np
import cv2
#from picamera import PiCamera
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative,Vehicle, LocationGlobal, Command
import time
from time import gmtime, strftime

# vehicle = connect("/dev/serial0", wait_ready=True, baud=921000)
vehicle = connect("udp:192.168.137.103:14550", wait_ready=True)
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
    # cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 5))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 38.6941650 ,35.4606727 , 5))#1
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 38.6942204 ,35.4605145 , 5))#2 Su alma alanı
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 38.6943038, 35.4603917 , 5))#3
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 38.6942703, 35.4603022 , 5))#4
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 38.6942441, 35.4602586 , 5))#5
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 38.6942012, 35.4602462 , 5))#6
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 38.6941599, 35.4602650 , 5))#7
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 38.6940453, 35.4605875 , 5))#8
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 38.6940568, 35.4606505 , 5))#9
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 38.6940890, 35.4606901 , 5))#10
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 38.6941342, 35.4606888 , 5))#11
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 38.6941650, 35.4606727 , 5))#12
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 38.6941650, 35.4606727 , 5))#12/

    print(" Upload new commands to vehicle")
    cmds.upload()

def second_tour(lat,lon):
    cmds = vehicle.commands
    cmds.clear()
    vehicle.flush() 
    # cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 5))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 38.6941650 ,35.4606727 , 5))#1
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 2, 0, 0, 0, 38.6942344674483 ,35.4605628736317 , 5))#2 Su alma alanı
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 3, 0, 0, 0, 38.6942344674483 ,35.4605628736317 , 1))#2 Su alma alanı
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 2, 0, 0, 0, 38.6942344674483 ,35.4605628736317 , 5))#2 Su alma alanı
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 2, 0, 0, 0, lat ,lon , 5))#3 Su bırakma alanı
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 3, 0, 0, 0, lat ,lon , 1))#3 Su bırakma alanı
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 38.6943038, 35.4603917 , 5))#3
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 38.6942703, 35.4603022 , 5))#4
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 38.6942441, 35.4602586 , 5))#5
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 38.6942012, 35.4602462 , 5))#6
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 38.6941599, 35.4602650 , 5))#7
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 38.6940453, 35.4605875 , 5))#8
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 38.6940568, 35.4606505 , 5))#9
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 38.6940890, 35.4606901 , 5))#10
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 38.6941342, 35.4606888 , 5))#11
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 38.6941650, 35.4606727 , 5))#12
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 38.6941650, 35.4606727 , 5))#12/
    
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

while vehicle.commands.next <=11:
    
    nextwaypoint=vehicle.commands.next
    if vehicle.commands.next == 3:
        ret, frame = cap.read()
        
        if ret == True:
            # Save the video
            out.write(frame)
            # Filter red color
            frame = cv2.bilateralFilter(frame,9,75,75)
            frame_hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
            mask1 = cv2.inRange(frame_hsv, (0, 70, 50), (10, 255, 255))
            mask2 = cv2.inRange(frame_hsv, (170, 70, 50), (180, 255, 255))
            mask = mask1 + mask2
            white_pixels = np.where(mask==255)
            cX = np.average(white_pixels[1])
            cY = np.average(white_pixels[0])
            r = cX*cX + cY*cY # need to be changed

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
if r_square:
    index = r_square.index(min(r_square))
    finalpos = pos[index]
    print(finalpos[0], finalpos[1])
    second_tour(finalpos[0],finalpos[1])
    vehicle.commands.next=0
    nextwaypoint=0

    while vehicle.commands.next <=16:
        nextwaypoint=vehicle.commands.next

print("Out")
vehicle.mode = VehicleMode("LAND")
