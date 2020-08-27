#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
from pymavlink import mavutil
from gpiozero import LED, Button

# Connecting to Pixhawk
#Sa
# Functions
# Arm and Take-off
def arm_and_takeoff(aTargetAltitude):
    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:      
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)      
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)

# Mission 1: Go to water intake area using specified route
def mission_1(aLocation, aSize):
    cmds = vehicle.commands
    print("Clear any existing commands")
    cmds.clear() 
    # Add way points
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 55, 55, 55))
    # Upload the mission
    cmds.upload()

# Mission 2: Scan for the water discharge area (image processing)
def mission_2():
    print("Area found")

# Mission 3: Go to water intake area using specified route
def mission_3():
    cmds = vehicle.commands
    print("Clear any existing commands")
    cmds.clear() 
    # Add way points
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 55, 55, 55))
    # Upload the mission
    cmds.upload()

# Water intake
def water_intake():
    print("water intake")

    # Descent up to 20 cm height from ground

    # Ckech if motors touching water and water level

    # Pump water

# Mission 4: Go to the water discharge area
def mission_4(GPS):
    cmds = vehicle.commands
    print("Clear any existing commands")
    cmds.clear() 
    # Add way points
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 55, 55, 55))
    # Upload the mission
    cmds.upload()

# Mission 5: Go back to the landing area
def mission_5():
    cmds = vehicle.commands
    print("Clear any existing commands")
    cmds.clear() 
    # Add way points
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 55, 55, 55))
    # Upload the mission
    cmds.upload()

# Water discharge
def water_discharge():
    print("water discharge")

    # Descent up to 20 cm from ground

    # Check water level

    # Discharge water


# Create and upload the Mission 1
mission_1()

# Automous take-off (5 m)
arm_and_takeoff(5)

# Reset mission set to first (0) waypoint
vehicle.commands.next = 0

# Set mode to AUTO to start mission
vehicle.mode = VehicleMode("AUTO")

# Check if Mission 1 is done
while vehicle.commands.next != 21:
    time.sleep(1)

# Scan for the water discharge area
water_discharge_GPS = mission_2()

# Check if Mission 2 is done
while not water_discharge_GPS:
    time.sleep(1)

# Create and upload the Mission 3
mission_3()

# Reset mission set to first (0) waypoint
vehicle.commands.next = 0

# Intaking water
water_intake()

# Control structures --------

# Go to the water discharge area
mission_4(water_discharge_GPS)

# Control structures --------

# Discharge water
water_discharge()

# Control structures --------

# Create and upload the Mission 5
mission_5()

# Reset mission set to first (0) waypoint
vehicle.commands.next = 0

# Land
