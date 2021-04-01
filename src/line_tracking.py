#!/usr/bin/env python3

import roslib
import sys
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs import Twist
from sensor_msgs import Image

class LaneTrack(object):

    def __init__(self):
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/raspicam_node/image/compressed",Image, self.camera_callback)

    def camera_callback(self.data):
        
        try:
            cv_image=self.bridge_object.imgmsg_to_cv2(data,desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        cv2.imshow("Image Window",cv_image)
        cv2.waitKey(1)

def main():
     lane_tracking_object=LineTrack()
     rospy.init_node("line_track_node", anonymous=True)
     try:
        rospy.spin()
     except KeyboardInterrupt:
        print("Shutting down")
     cv2.destroyAllWindows()

if __name__ =='__main__':
        main()
        

