from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
from pymavlink import mavutil
from gpiozero import LED, Button
vehicle = connect('/dev/serial0', wait_ready=True, baud=921000)

def arm_and_takeoff(aTargetAltitude):
    while not vehicle.is_armable:
        time.sleep(1)

    vehicle.mode    = VehicleMode("GUIDED")
    vehicle.armed   = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        time.sleep(1)

    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        #Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            break
        time.sleep(1)

arm_and_takeoff(1)