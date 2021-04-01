#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time
import numpy as np

class AexrosMoves():
    def __init__(self):
        # positions
        
        self.odom_sub = rospy.Subscriber('/odom', Odometry,self.odomCb)
        # velocities
      
        self.vel_pub = rospy.Publisher('/rosbot/cmd_vel', Twist, queue_size=5)
        self.vel_msg = Twist()
        # initial orientation
        self.yaw = 0
        
    def odomCb(self,odom_msg):
        self.current_position = odom_msg.pose.pose
        self.yaw = get_angle_from_pose(self.current_position)

    def turn(self,target_angle):
        print("yaw angle: {}".format(self.yaw/np.pi*180))
        # convert target angle from degrees to radians
        target_angle = target_angle*np.pi/180
        # compare target angle with actual angle (yaw) of the robot
        error_angle = target_angle - self.yaw
        # calculate angular command
        command_angle = 5 * error_angle
        # give 0 linear velocity
        command_vel = 0.0
        # publish the linear and angular velocity commands
        self.publish_vel(command_vel,command_angle)
   
    def publish_vel(self,vx,wz):
        self.vel_msg.linear.x = vx
        self.vel_msg.angular.z = wz
        self.vel_pub.publish(self.vel_msg)
        
def get_angle_from_pose(pose):
    orient_list = [pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]
    (roll,pitch,yaw) = euler_from_quaternion(orient_list)
    return yaw
        
if __name__ == '__main__':
    # init node
    node_name = 'move_turn'
    rospy.init_node(node_name, anonymous=True)
    # Rate of publishing
    rate = rospy.Rate(10)

    # call Turtlebot class
    mover = AexrosMoves()
    
    rospy.loginfo('Initializing robot commands ............................')
    time.sleep(2)

    while not rospy.is_shutdown():
        ####################################################################################################
          # insert the desired degrees here
        
        mover.turn(0)
        
        ####################################################################################################
        rate.sleep()
