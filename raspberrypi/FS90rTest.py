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
    pwm.ChangeDutyCycle(10) # for cw use (5)
    time.sleep(2)
pwm.stop()
GPIO.cleanup()