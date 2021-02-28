import RPi.GPIO as GPIO
from time import sleep


GPIO.setmode(GPIO.BCM)
GPIO.setup(16, GPIO.OUT)

pwm= GPIO.PWM(16,50)
pwm.start(0)

pwm.ChangeDutyCycle(5)
sleep(0.5)
pwm.ChangeDutyCycle(7.5)
sleep(0.5)
pwm.ChangeDutyCycle(10)
sleep(0.5)

pwm.stop()
GPIO.cleanup()
