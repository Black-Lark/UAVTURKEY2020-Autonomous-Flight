import numpy as np
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative,Vehicle, LocationGlobal
import time
import math
# Connect
vehicle = connect("/dev/serial0", wait_ready=True, baud=921000)

def arm_and_takeoff(aTargetAltitude):
 
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
    vehicle.simple_takeoff(aTargetAltitude)  

    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.90:
            print("Reached target altitude")
            break
        time.sleep(1)


arm_and_takeoff(2)
time.sleep(1)
vehicle.mode = VehicleMode("LAND")
while vehicle.rangefinder.distance >= 0.25:
    print(vehicle.rangefinder.distance)

vehicle.mode = VehicleMode("LOITER")
