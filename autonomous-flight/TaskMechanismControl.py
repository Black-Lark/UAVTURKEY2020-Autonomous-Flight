from gpiozero import LED, Button
from signal import pause 
import RPi.GPIO as GPIO
import time

# Water sensor
level0 = Button(6) # Pompa motors
level1 = Button(13) # Kabın içerisinde zeminde 
level2 = Button(19) # kabın içerisinde yukarıda 
level3 = Button(26) # hava valfinin orada

pump_motor_relay = LED(7)
dc_motor_relay = LED(8)
#GPIO.setwarnings(False)
#GPIO.setmode(GPIO.BOARD)
#GPIO.setup(33, GPIO.OUT)
#pwm=GPIO.PWM(33, 50)
print("Starting")
while True:
    if level0.is_pressed is True:
        pump_motor_relay.off()
        break

time.sleep(10)
pump_motor_relay.on()
dc_motor_relay.off()
time.sleep(5)
dc_motor_relay.on()
print("finished")

# t0 = time.clock()
# t0 = time.time()

# while time.time() - t0 < 2:
#     dc_motor_relay.off()    

# dc_motor_relay.on()

#time.sleep(2)

#dc_motor_relay.off()


# while True:
#     print("Level 0: ", level0.is_pressed)
#     print("Level 1: ", level1.is_pressed)
#     print("Level 2: ", level2.is_pressed)
#     print("Level 3: ", level3.is_pressed)
#     time.sleep(1)



# dcmotor = LED(21)

# dcmotor.on()

#pause()