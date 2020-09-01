import RPi.GPIO as GPIO
import time
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(13, GPIO.OUT)

GPIO.setmode(GPIO.BOARD)
GPIO.setup(13, GPIO.OUT)
pwm=GPIO.PWM(13, 50)
pwm.start(0)

pwm.ChangeDutyCycle(5) # left -90 deg position
time.sleep(2)
pwm.stop()
GPIO.cleanup()