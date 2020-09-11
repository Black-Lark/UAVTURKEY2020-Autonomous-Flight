from __future__ import print_function
import numpy as np
import cv2
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative,Vehicle, LocationGlobal, Command
from time import gmtime, strftime 
import time
import math

vehicle = connect("/dev/serial0", wait_ready=True, baud=921000)

def current_yaw():
    yaw = math.degrees(vehicle.attitude.yaw)
    if yaw < 0:
        yaw = yaw + 360
    return yaw

while True:
    print(current_yaw())
    time.sleep(0.5)