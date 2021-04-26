#!/usr/bin/env python3

import roslib
import sys
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int16


class LineFollow(object):

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/raspicam_node/image/compressed",CompressedImage, self.camera_callback)
        print(" << Subscribe image from camera")
        self.vel_pub = rospy.Publisher("rosbot/cmd_vel", Twist, queue_size = 5)
        print(" >> Publish the velocity ")
        self.vel = Twist()
        self.servo_pub = rospy.Publisher("servo", Int16, queue_size = 2)
        print(" >> Publish the servo value")
        self.servo_angle = Int16()
       

    def stop(self):
        self.vel.linear.x = 0
        self.vel.angular.z = 0
        

    def drive(self,vx,wz) :
        self.vel.linear.x = vx
        self.vel.angular.z = wz
        
        
    def servo(self,angle):
        self.servo_angle.data = angle 

    def camera_callback(self,data):
        self.servo(-30)
        
        try:
            cv_image=self.bridge.compressed_imgmsg_to_cv2(data,"bgr8")
            self.vel_pub.publish(self.vel)
            self.servo_pub.publish(self.servo_angle)
        except CvBridgeError as e:
            print(e)

        blob_params = None
        #blur to reduce noise
        #blurred = cv2.GaussianBlur(cv_image, (11,11),0)
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV).astype(np.float)      

        lower_color= np.array([0,127,33])
        upper_color = np.array([360,216,140])

        mask = cv2.inRange(hsv,lower_color,upper_color)  

         #find contour
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
       

        #erode
        mask = cv2.erode(mask, None, iterations=2)

        #dilate area larger
        mask = cv2.dilate(mask, None, iterations=2)
       
        

        #image size
        rows = cv_image.shape[0]
        cols = cv_image.shape[1]
        size = min([rows,cols])
        center_x = int(cols/2.0)
        center_y = int(rows/2.0)
        
        m = cv2.moments(mask, False)
        try:
            cx,cy = m['m10']/m['m00'], m['m10']/m['m00']
        except ZeroDivisionError:
            cy,cx = rows/2,cols/2
        cv2.circle(cv_image,(int(cx),int(cy)), 10,(0,255,0),-1)
        cv2.putText(cv_image, ' C ',(int(cx),int(cy)),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,0),2, cv2.LINE_AA)
        
        err_x = cx-center_x
        wz = -err_x/150
        
        self.drive(0.33,wz)
        
        rospy.loginfo("angular vel : "+str(wz))
        #Cannot detect ball
        #if object_h_pixel == 0:
        #    hdif = 0
        #    self.stop()
        #elif (object_h_pixel < left_bound and radius > 65):
        #    #if (radius > 70 and object_h_pixel < 85):
        ##    vx = 0
        #    wz = 5.8
        #    self.drive(vx,wz)#
        #
        #elif (object_h_pixel > right_bound and radius > 65):
        #    #if (radius > 70 and object_h_pixel > 325):
        #    vx = 0
        #    wz = -5.8
        #    self.drive(vx,wz)
        #else:
        #    if (radius<60):
        #        vx = 0.45
        #        wz = 0
        #        self.drive(vx,wz)
        #    else:
        #        self.stop()
            
        
        
      
        cv2.imshow("Image Window",cv_image)
        cv2.imshow("MASK",mask)
       
      
   
     
        cv2.waitKey(1)

def main():
     
     rospy.init_node("line_following_node", anonymous=True)
     line_following_object=LineFollow()
     try:
        rospy.spin()
     except KeyboardInterrupt:
        print("Shutting down")
     cv2.destroyAllWindows()

if __name__ =='__main__':
        main()
        

