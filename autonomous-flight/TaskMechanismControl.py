from gpiozero import LED, Button
from signal import pause 
import RPi.GPIO as GPIO
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative,Vehicle, LocationGlobal, Command
#vehicle = connect("/dev/serial0", wait_ready=True, baud=921000)
# Water sensor
#level0 = Button(6) # Pompa motors
#level1 = Button(13) # Kabın içerisinde zeminde 
#level2 = Button(19) # kabın içerisinde yukarıda 
#level3 = Button(26) # hava valfinin orada

# pump_motor_relay = LED(7)
# pump_motor_relay.on()
dc_motor_relay = LED(8)
dc_motor_relay.on()
#GPIO.setwarnings(False)
#GPIO.setmode(GPIO.BOARD)
#GPIO.setup(33, GPIO.OUT)
#pwm=GPIO.PWM(33, 50)
# print("Starting")
# while True:
#     print(vehicle.rangefinder.distance)
#     if vehicle.rangefinder.distance < 0.30:
#         pump_motor_relay.off()
#         break
# time.sleep(17)
# pump_motor_relay.on()
# dc_motor_relay.off()
# time.sleep(10)
# dc_motor_relay.on()


# t0 = time.clock()
# t0 = time.time()

# while time.time() - t0 < 2:
#     dc_motor_relay.off()    

dc_motor_relay.on()

time.sleep(2)

dc_motor_relay.off()

time.sleep(3)
print("finished")

# while True:
#     print("Level 0: ", level0.is_pressed)
#     print("Level 1: ", level1.is_pressed)
#     print("Level 2: ", level2.is_pressed)
#     print("Level 3: ", level3.is_pressed)
#     time.sleep(1)



# dcmotor = LED(21)

# dcmotor.on()

#pause()