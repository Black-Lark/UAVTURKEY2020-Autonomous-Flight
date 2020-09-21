from gpiozero import LED, Button
from signal import pause 
import RPi.GPIO as GPIO
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative,Vehicle, LocationGlobal, Command
vehicle = connect("/dev/serial0", wait_ready=True, baud=921000)

pump_motor_relay = LED(7)
pump_motor_relay.on()
dc_motor_relay = LED(8)
dc_motor_relay.on()

print("Starting")
process_start = time.time()
pump_is_on = 0
water_taken_start = 0
water_taken = 0

while ((time.time() - process_start) < 20) and water_taken < 12:
    #print(vehicle.rangefinder.distance)
    print(water_taken)
    if (vehicle.rangefinder.distance < 0.35) and (pump_is_on == 0):
        if water_taken_start == 0:
            water_taken_start = time.time()
        pump_motor_relay.off()
        pump_is_on = 1
        time.sleep(0.25)
    else:
        water_taken = water_taken + time.time() - water_taken_start
        water_taken_start = 0
        pump_motor_relay.on()
        pump_is_on = 0


