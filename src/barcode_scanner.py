#!/usr/bin/env python3

import roslib
import sys
import rospy
import numpy as np
import cv2
from pyzbar import pyzbar
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage



class Scanner(object):

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/raspicam_node/image/compressed",CompressedImage, self.camera_callback)
        print(" << Subscribe image from camera")
        

    def camera_callback(self,data):
        # Connect ROS with OpenCV via ROS cv_bridge package
        try:
            cv_image=self.bridge.compressed_imgmsg_to_cv2(data,"bgr8")
  
        except CvBridgeError as e:
            print(e)  
      
        self.scan(cv_image)
    
        
    def scan(self, image):
        im=image
        barcodes = pyzbar.decode(im)
        for barcode in barcodes:
            (x, y, w, h) = barcode.rect
            cv2.rectangle(im, (x, y), (x + w, y + h), (0, 0, 255), 2)
            barcodeData = barcode.data.decode("utf-8")
            barcodeType = barcode.type
            # draw the barcode data and barcode type on the image
            text = "{} ({})".format(barcodeData, barcodeType)
            cv2.putText(im, text, (x, y - 10),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            print("Text", text)
        cv2.imshow("Image", im)
        cv2.waitKey(1)

def main():
     
     rospy.init_node("Scanner_node", anonymous=True)
     dt=Scanner()
     try:
        rospy.spin()
     except KeyboardInterrupt:
        print("Shutting down")
     cv2.destroyAllWindows()

if __name__ =='__main__':
        main()
        

