import numpy as np
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative,Vehicle, LocationGlobal, Command
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
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.90:
            print("Reached target altitude")
            break
        time.sleep(1)

def auto_altitude():
    cmds = vehicle.commands
    cmds.clear()
    vehicle.flush()
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 2))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 1, 0, 0, 0, 38.6939372, 35.4610946 , 2))# 2 m
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 1, 0, 0, 0, 38.6939372, 35.4610946 , 1))# 2 m
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 1, 0, 0, 0, 38.6939372, 35.4610946 , 0.5))# 2 m
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 1, 0, 0, 0, 38.6939372, 35.4610946 , 0.25))# 2 m
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 1, 0, 0, 0, 38.6939372, 35.4610946 , 0.15))# 2 m

    cmds.upload()

arm_and_takeoff(2)
auto_altitude()
vehicle.commands.next = 0
vehicle.mode = VehicleMode("AUTO")
time.sleep(1)
while vehicle.rangefinder.distance > 0.25:
    nextwaypoints = vehicle.commands.next

vehicle.mode = VehicleMode("LOITER")
