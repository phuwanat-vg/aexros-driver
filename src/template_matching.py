#!/usr/bin/env python3

import roslib
import sys
import rospy
import numpy as np
import cv2
import imutils
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage



class Detector(object):

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/raspicam_node/image/compressed",CompressedImage, self.camera_callback)
        print(" << Subscribe image from camera")
        self.vel_pub = rospy.Publisher("rosbot/cmd_vel", Twist, queue_size = 5)
        print(" >> Publish the velocity ")
        self.vel = Twist()
        

    def camera_callback(self,data):
        # Connect ROS with OpenCV via ROS cv_bridge package
        try:
            cv_image=self.bridge.compressed_imgmsg_to_cv2(data,"bgr8")
            self.vel_pub.publish(self.vel)
        except CvBridgeError as e:
            print(e)  
      
        self.detect(cv_image)
    
        
    def detect(self, image):
        startX = 0
        startY = 0
        im = image
        template = cv2.imread('/home/ubuntu/aexros_ws/src/aexros_driver/target/coke_logo.png')
        # convert both the image and template to grayscale
        imageGray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        templateGray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
        result = cv2.matchTemplate(imageGray, templateGray,cv2.TM_CCOEFF_NORMED)
        (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(result)
        # determine the starting and ending (x, y)-coordinates of the
        # bounding box
        (startX, startY) = maxLoc
        endX = startX + template.shape[1]
        endY = startY + template.shape[0]
        # draw the bounding box on the image
        cv2.rectangle(im, (startX, startY), (endX, endY), (255, 0, 0), 3)
     
        #cv2.imshow("Template", template)
        cv2.imshow("Image", im)
        cv2.waitKey(1)

def main():
     
     rospy.init_node("shape_count_node", anonymous=True)
     dt=Detector()
     try:
        rospy.spin()
     except KeyboardInterrupt:
        print("Shutting down")
     cv2.destroyAllWindows()

if __name__ =='__main__':
        main()
        

