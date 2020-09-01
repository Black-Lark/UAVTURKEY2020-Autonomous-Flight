import RPi.GPIO as GPIO
from time import sleep
GPIO.setmode(GPIO.BOARD)
GPIO.setup(13, GPIO.OUT)

GPIO.setmode(GPIO.BOARD)
GPIO.setup(13, GPIO.OUT)

pwm=GPIO.PWM(13, 50)
pwm.start(0)

pwm.ChangeDutyCycle(5) # left -90 deg position

pwm.stop()
GPIO.cleanup()