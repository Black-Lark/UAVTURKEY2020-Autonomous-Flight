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
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 1, 0, 0, 0, 38.6939188797346, 35.4611829668283 , 2))# 2 m
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 1, 0, 0, 0, 38.6939188797346, 35.4611829668283 , 1))# 1 m
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 1, 0, 0, 0, 38.6939188797346, 35.4611829668283 , 0.5))# 0.5 m
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 1, 0, 0, 0, 38.6939188797346, 35.4611829668283 , 0.25))# 0.25 m
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 1, 0, 0, 0, 38.6939188797346, 35.4611829668283 , 0.15))# 0.15 m


arm_and_takeoff(2)
auto_altitude()
vehicle.commands.next = 0
vehicle.mode = VehicleMode("AUTO")
time.sleep(1)
while vehicle.rangefinder.distance > 0.25:
    nextwaypoints = vehicle.commands.next

vehicle.mode = VehicleMode("LOITER")
