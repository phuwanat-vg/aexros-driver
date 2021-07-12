#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3, PoseStamped, WrenchStamped
from mpu9250_i2c import *
import time
import sys


def imu_talker():
    pub = rospy.Publisher('imu', Imu, queue_size = 5)
    rospy.init_node('aexros_imu', anonymous = False)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        ax,ay,az,wx,wy,wz = mpu6050_conv() # read and convert mpu6050 data
        mx,my,mz = AK8963_conv() # read and convert AK8963 magnetometer data
        groll = wx
        gpitch = wy
        gyaw = 0.56
        ax = ax
        ay = ay
        az = az-0.3
        imu_msg = Imu()
        #imu_msg.header.stamp = rospy.rostime.Time()
        imu_msg.angular_velocity.x = groll/180*3.14
        imu_msg.angular_velocity.y = gpitch/180*3.14
        imu_msg.angular_velocity.z = gyaw/180*3.14
        imu_msg.linear_acceleration.x = ax 
        imu_msg.linear_acceleration.y = ay
        imu_msg.linear_acceleration.z = az
        imu_str = "gx:{:.3f} gy:{:.3f} gz:{:.3f} ax:{:.3f} ay:{:.3f} az:{:.3f}"
        print("unit of g : rps , unit of a : g")
        print(imu_str.format(groll,gpitch,gyaw,ax,ay,az))
        pub.publish(imu_msg)
        rate.sleep()
                        
if __name__ == '__main__':
    try: 
        imu_talker()
    except rospy.ROSInterruptException:
        pass
