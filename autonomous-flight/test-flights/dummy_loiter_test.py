from __future__ import print_function
import numpy as np
import cv2
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative,Vehicle, LocationGlobal, Command
from time import gmtime, strftime 
import time
import math
from gpiozero import LED, Button
from signal import pause 
import RPi.GPIO as GPIO

#vehicle = connect("tcp:127.0.0.1:5762", wait_ready=True)
vehicle = connect("/dev/serial0", wait_ready=True, baud=921000)
# OpenCV
level0 = Button(6) # Pompa motors
pump_motor_relay = LED(7)
pump_motor_relay.on()
dc_motor_relay = LED(8)
dc_motor_relay.on()
cap = cv2.VideoCapture(0)
fourcc = cv2.VideoWriter_fourcc(*'XVID')
file_name = strftime("%Y-%m-%d_%H-%M-%S", gmtime()) + ".avi"
out = cv2.VideoWriter(file_name,fourcc, 25, (640,480))

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

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.90:
            print("Reached target altitude")
            break
        time.sleep(1)


def get_location_metres(original_location, dNorth, dEast):
    earth_radius = 6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    if type(original_location) is LocationGlobal:
        targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
    else:
        raise Exception("Invalid Location object passed")
    return targetlocation


def get_distance_metres(aLocation1, aLocation2):
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


def goto(dNorth, dEast, alt, gotoFunction=vehicle.simple_goto):
    currentLocation=vehicle.location.global_relative_frame
    currentLocation.alt = alt
    targetLocation=get_location_metres(currentLocation, dNorth, dEast)
    targetDistance = get_distance_metres(currentLocation, targetLocation)
    gotoFunction(targetLocation)

    while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
        remainingDistance=get_distance_metres(vehicle.location.global_frame, targetLocation)
        print ("Distance to target: ", remainingDistance)
        if remainingDistance<=targetDistance*0.5: #Just below target, in case of undershoot.
            print ("Reached target")
            break
        

def distance_estimate(alt, deviation):
    alt = alt * 100
    a = 0.002
    b = -0.0129
    pixel_to_cm = a * alt + b
    return deviation * pixel_to_cm / 100


def current_yaw():
    yaw = math.degrees(vehicle.attitude.yaw)
    if yaw < 0:
        yaw = yaw + 360
    return yaw



def field_detection_tour(): 
    cmds = vehicle.commands
    cmds.clear()
    vehicle.flush()
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 5))# Dummy
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 36.9778751 ,37.3032403 , 5))# 2
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 36.9778590 ,37.3033006 , 5))# 3 Su alma alanı
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 36.9778590 ,37.3033006 , 5))# 3 Su alma alanı
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 1, 0, 0, 0, 36.9778590 ,37.3033006 , 4))# 3 Su alma alanı
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 1, 0, 0, 0, 36.9778590 ,37.3033006 , 2))# 3 Su alma alanı
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 2, 0, 0, 0, 36.9778590 ,37.3033006 , 1))# 3 Su alma alanı
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 2, 0, 0, 0, 36.9778590 ,37.3033006 , 0.75))# 3 Su alma alanı
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 2, 0, 0, 0, 36.9778590 ,37.3033006 , 0.5))# 3 Su alma alanı
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 2, 0, 0, 0, 36.9778590 ,37.3033006 , 0.4))# 3 Su alma alanı
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 2, 0, 0, 0, 36.9778590 ,37.3033006 , 0.4))# 3 Su alma alanı

    print(" Upload new commands to vehicle")
    cmds.upload()

arm_and_takeoff(5)
# Calling field_detection tour
field_detection_tour()
vehicle.commands.next = 0
nextwaypoint = 0

while vehicle.mode != "AUTO":
    vehicle.mode = VehicleMode("AUTO")
    print("waiting auto mode")


while vehicle.commands.next <=9:
    
    nextwaypoint= vehicle.commands.next 
    print(vehicle.rangefinder.distance)
    if vehicle.rangefinder.distance < 0.5:
        pump_motor_relay.off()
        print("pump motor started")
        break


while vehicle.mode != "LOITER":
    vehicle.mode = VehicleMode("LOITER")
    print("waiting loiter mode")

time.sleep(5) # 5 saniye su alma
pump_motor_relay.on()

print("finished")