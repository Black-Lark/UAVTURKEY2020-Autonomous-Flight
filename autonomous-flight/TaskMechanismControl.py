from gpiozero import LED, Button
import time

level0 = Button(31)
level1 = Button(33)
level2 = Button(35)
level3 = Button(37)


while True:
    print("Level 0: " + level0)
    print("Level 1: " + level1)
    print("Level 2: " + level2)
    print("Level 3: " + level3)
    time.sleep(1)
