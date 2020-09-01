import RPi.GPIO as GPIO
import time
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(33, GPIO.OUT)

GPIO.setmode(GPIO.BOARD)
GPIO.setup(33, GPIO.OUT)
pwm=GPIO.PWM(33, 50)
pwm.start(0)
while True:
    pwm.ChangeDutyCycle(5) # left -90 deg position
    time.sleep(2)
pwm.stop()
GPIO.cleanup()