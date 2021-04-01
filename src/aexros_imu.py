#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3, PoseStamped, WrenchStamped
import FaBo9Axis_MPU9250
import time
import sys

mpu9250 = FaBo9Axis_MPU9250.MPU9250()

def imu_talker():
            pub = rospy.Publisher('imu', Imu, queue_size = 5)
            rospy.init_node('aexros_imu', anonymous = False)
            rate = rospy.Rate(10)
            
            
             
            while not rospy.is_shutdown():
                    gyro = mpu9250.readGyro()
                    accel = mpu9250.readAccel()
                
                    groll = gyro['x']
                    gpitch = gyro['y']
                    gyaw = gyro['z']
                    ax = accel['x']
                    ay = accel['y']
                    az = accel['z']
                
                    
                    imu_msg = Imu()
                    #mu_msg.header.stamp = rospy.rostime.Time()
                    imu_msg.angular_velocity.x = groll
                    imu_msg.angular_velocity.y = gpitch
                    imu_msg.angular_velocity.z = gyaw-0.79
                    imu_msg.linear_acceleration.x = ax  
                    imu_msg.linear_acceleration.y = ay
                    imu_msg.linear_acceleration.z = az  
                        
    
                
                    imu_str = "gx = {} gy = {} gz = {} ax= {} ay = {} az = {}"
                
                    rospy.loginfo(imu_str.format(groll,gpitch,gyaw,ax,ay,az))
                
                    pub.publish(imu_msg)
                    rate.sleep()
                        
if __name__ == '__main__':
    try: 
        imu_talker()
    except rospy.ROSInterruptException:
        pass
