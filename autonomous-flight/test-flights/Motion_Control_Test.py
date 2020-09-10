from __future__ import print_function
import numpy as np
import cv2
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative,Vehicle, LocationGlobal, Command
from time import gmtime, strftime 
import time
import math

#vehicle = connect("tcp:127.0.0.1:5762", wait_ready=True)
vehicle = connect("/dev/serial0", wait_ready=True, baud=921000)

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

while True:
    if vehicle.location.global_relative_frame>0.5:
        vehicle.mode= VehicleMode("LAND")
    else:
        vehicle.mode = VehicleMode("GUIDED")

