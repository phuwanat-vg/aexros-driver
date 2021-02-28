#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
from time import sleep
import numpy as np

def servo_control(angle):
    rospy.loginfo('{}'.format(angle.data))
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(16,GPIO.OUT)
    pwm = GPIO.PWM(16,50)
    pwm.start(0)    

    angle = angle.data
    duty = angle/18.0+7.5
    GPIO.output(16,True)
    pwm.ChangeDutyCycle(duty)
    sleep(1)
    GPIO.output(16,False)
    pwm.ChangeDutyCycle(duty)
    pwm.stop()
    GPIO.cleanup() 

def get_angle():
    rospy.init_node('servo_node', anonymous = True)
    rospy.Subscriber("servo", Float32, servo_control)
    rospy.spin()

if __name__=='__main__':
    try:
   
     
        get_angle()
       
    except KeyboardInterrupt:
        print("Keyboard Interrupt")
 



