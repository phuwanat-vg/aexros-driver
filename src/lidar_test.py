#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def laser_callback(msg):
    array_number = len(msg.ranges)
    p1= msg.ranges[1]
    p2= msg.ranges[90]
    
    vel_msg=Twist()

    range_array = {'point1':p1,'point2':p2}
    rospy.loginfo("point1: %f point2 : %f length: %f",p1,p2,array_number)

    linear_x = 0.5
    angular_z = 0
    vel_msg.linear.x = linear_x
    vel_msg.angular.z = angular_z
    pub.publish(vel_msg)
def main():
    global pub
    rospy.init_node('Laser_Scan_Data_Loading')
    sub= rospy.Subscriber("/scan", LaserScan, laser_callback) 
    pub = rospy.Publisher('/rosbot/cmd_vel', Twist, queue_size=5) 
    rospy.spin()

if __name__ == '__main__':
    main()

