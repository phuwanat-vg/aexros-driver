#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def laser_callback(msg):
    array_number = len(msg.ranges)
    front = msg.ranges[361]
    right = msg.ranges[270]
    left = msg.ranges[450]
    range_array = {'front' : front, 'left' : left,'right' : right}

    rospy.loginfo("number of array :%f",array_number)
    rospy.loginfo("front : %f left : %f right : %f ",front,left,right)

    avoidance(range_array)

def avoidance(range_array):
    
    vel_msg = Twist()
    linear_x=0
    angular_z=0
   
    if range_array['front']!=0 and range_array['left']!=0 and range_array['right']!=0:
        if range_array['front']>1.0 and range_array['left']>1.00 and range_array['right']>1.0:
            robot_status = 'forward'
            linear_x = 0.4
            angular_z = 0
        elif range_array['front']<1.0 and range_array['left']>1.0 and range_array['right']>1.0:
            robot_status = 'turn left'
            linear_x = 0.4
            angular_z = 5
        elif range_array['front']>1.0 and range_array['left']<1.0 and range_array['right']>1.0:
            robot_status = 'turn right'
            linear_x = 0.4
            angular_z = -5
        elif range_array['front']>1.0 and range_array['left']>1.0 and range_array['right']<1.0:
            robot_status = 'turn left'
            linear_x = 0.4
            angular_z = 5
        else:
            robot_status = 'unknow'
            linear_x = 0
            angular_z = 5
        rospy.loginfo(robot_status)  

        vel_msg.linear.x = linear_x
        vel_msg.angular.z = angular_z
        pub.publish(vel_msg)
    else:
        vel_msg.linear.x = 0.4
        vel_msg.angular.z = 0
        pub.publish(vel_msg)

def main():
    global pub
    rospy.init_node('Laser_Scan_Data_Loading')
    sub= rospy.Subscriber("/scan", LaserScan, laser_callback)
    pub = rospy.Publisher('/rosbot/cmd_vel', Twist, queue_size=5) 
    rospy.spin()

if __name__ == '__main__':
    main()

