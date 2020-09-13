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
while (level2.is_pressed and level3.is_pressed) is False:
    if level0 is True:
        pump_motor_relay.off()
        print("level1: ", level1.is_pressed)
        print("level2: ", level2.is_pressed)
print("finished")
pump_motor_relay.on()
#t0 = time.clock()
#t0 = time.time()

#while time.time() - t0 < 2:
    #pwm.ChangeDutyCycle(10) # for cw use (5)    


#dc_motor_relay = LED(8)
#dc_motor_relay.on()

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