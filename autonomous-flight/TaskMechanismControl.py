from gpiozero import LED, Button
from signal import pause 
import time

level0 = Button(6)
level1 = Button(13)
level2 = Button(19)
level3 = Button(26)

dc_motor_relay = LED(16)
dc_motor_relay.on()

time.sleep(2)

dc_motor_relay.off()


# while True:
#     print("Level 0: ", level0.is_pressed)
#     print("Level 1: ", level1.is_pressed)
#     print("Level 2: ", level2.is_pressed)
#     print("Level 3: ", level3.is_pressed)
#     time.sleep(1)



# dcmotor = LED(21)

# dcmotor.on()

pause()