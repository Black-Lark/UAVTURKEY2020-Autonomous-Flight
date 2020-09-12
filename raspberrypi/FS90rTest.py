from gpiozero import LED, Button
from signal import pause 
import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(32, GPIO.OUT)
pwm=GPIO.PWM(32, 50)
pwm.start(0)
t0 = time.clock()
t0 = time.time()

while time.time() - t0 < 2:
    pwm.ChangeDutyCycle(10) # for cw use (5)  

pwm.stop()
GPIO.cleanup()