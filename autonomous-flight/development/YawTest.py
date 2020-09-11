from __future__ import print_function
import numpy as np
import cv2
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative,Vehicle, LocationGlobal, Command
from time import gmtime, strftime 
import time
import math

# vehicle = connect("/dev/serial0", wait_ready=True, baud=921000)

def current_yaw():
    # yaw = math.degrees(vehicle.attitude.yaw)
    yaw = 60
    if yaw < 0:
        yaw = yaw + 360
    return yaw

# while True:
#     print(current_yaw())
#     time.sleep(0.5)

x = 10
y = -2

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
