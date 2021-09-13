#! /usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


def laser_callback(msg):
    array_number = len(msg.ranges)
    r_range = msg.ranges[27:160]
    fr_range = msg.ranges[161:293]
    f_range = msg.ranges[294:416]
    fl_range = msg.ranges[417:559]
    l_range = msg.ranges[560:665]
    
  
    fr = min((min(fr_range for fr_range in fr_range if fr_range > 0)),10)
    f = min((min(f_range for f_range in f_range if f_range > 0)),10)
    fl = min((min(fl_range for fl_range in fl_range if fl_range > 0)),10)
 
    
    range_array = {'front' : f, 'fleft' : fl,'fright' : fr}

    #rospy.loginfo("number of array :%f",array_number)
    rospy.loginfo("front : %f fleft : %f fright : %f ",range_array['front'],range_array['fleft'],range_array['fright'])
    #rospy.loginfo(min(msg.ranges[1:10]))
 
   
    
    #rospy.loginfo(fr)

    avoidance(range_array)

def avoidance(range_array):
    rate = rospy.Rate(10)
    vel_msg = Twist()
    linear_x=0
    angular_z=0
    th = 0.4
    
    if range_array['front']!=0 and range_array['fleft']!=0 and range_array['fright']!=0:
        if range_array['front'] > th and range_array['fleft'] > th and range_array['fright'] > th:
            robot_status = 'Forward'
            linear_x = 0.4
            angular_z = 0#-5.5
        elif range_array['front'] < th and range_array['fleft'] > th and range_array['fright'] > th:
            robot_status = 'Turn Right'
            linear_x = 0
            angular_z = 5.5
        elif range_array['front'] > th and range_array['fleft'] > th and range_array['fright'] < th:
            robot_status = 'case 3 - fright'
            linear_x = 0.5
            angular_z = 0
        elif range_array['front'] > th and range_array['fleft'] < th and range_array['fright'] > th:
            robot_status = 'case 4 - fleft'
            linear_x = 0#0.4
            angular_z = -5.5
        elif range_array['front'] < th and range_array['fleft'] > th and range_array['fright'] < th:
            robot_status = 'case 5 - front and fright'
            linear_x = 0
            angular_z = 5.5
        elif range_array['front'] < th and range_array['fleft'] < th and range_array['fright'] > th:
            robot_status = 'case 6 - front and fleft'
            linear_x = 0
            angular_z = 5.5
        elif range_array['front'] < th and range_array['fleft'] < th and range_array['fright'] < th:
            robot_status = 'case 7 - front and fleft and fright'
            linear_x = 0
            angular_z = 5.5
        elif range_array['front'] > th and range_array['fleft'] < th and range_array['fright'] < th:
            robot_status = 'case 8 - fleft and fright'
            linear_x = 0
            angular_z = 5.5#-5.5
        else:
            robot_status = 'Unknow'
            
     
        rospy.loginfo(robot_status)
        
       
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

