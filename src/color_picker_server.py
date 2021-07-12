#!/usr/bin/env python3

import roslib
import sys
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from dynamic_reconfigure.server import Server
from aexros_driver.cfg import color_pickerConfig


def callback(config,level):
    #rospy.loginfo("""Reconfigure request: {hue_l}, {hue_h},{saturation_l},{saturation_h},{lightness_l},{lightness_h}""".format(**config))
    print("{}".format(config))   
    return config
          

if __name__ =='__main__':
     rospy.init_node("color_picker_server_node", anonymous=False)
     print("Rqt reconfigure server start")
     
     srv = Server(color_pickerConfig, callback)
     try:
        rospy.spin()
     except KeyboardInterrupt:
        print("Shutting down")
        

