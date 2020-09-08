import numpy as np
import cv2
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative,Vehicle, LocationGlobal, Command
import time
import math

# vehicle = connect("tcp:127.0.0.1:5762", wait_ready=True)
vehicle = connect("/dev/serial0", wait_ready=True, baud=921000)

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
    gotoFunction(targetLocation)

    while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
        remainingDistance=get_distance_metres(vehicle.location.global_frame, targetLocation)
        print ("Distance to target: ", remainingDistance)
        if remainingDistance<=0.4: #Just below target, in case of undershoot.
            print ("Reached target")
            break
        time.sleep(2)


# Takeoff
arm_and_takeoff(2)

# Mission is starting
print("Mission is starting")
goto(0, -10, 2)
print("Wp #1 reached")
goto(1, 0, 2)
print("Wp #2 reached")
goto(0, 10, 2)
print("Wp #3 reached")
goto(-1, 0, 2)
print("Wp #4 reached - Exit")

