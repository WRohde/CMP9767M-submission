#!/usr/bin/python

import rospy
import sys
import numpy as np
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:
    def __init__(self):
        self.image_pub = rospy.Publisher("{}/opencv_image".format(robot_name),Image,queue_size=0)
        self.green_detected_pub = rospy.Publisher("{}/green_detected".format(robot_name),String,queue_size=0)
        self.image_sub = rospy.Subscriber("/{}/kinect2_camera/hd/image_color_rect".format(robot_name),Image,self.callback)
        self.bridge = CvBridge()

    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")
        except CvBridgeError as e:
            print(e)
        
        output,mask = self.green_mask(cv_image)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(output, "bgr8"))
            if np.sum(mask) > 1e6: #this threshold was chosen arbitrarily
                self.green_detected_pub.publish("TRUE")
            else:
                self.green_detected_pub.publish("FALSE")

        except CvBridgeError as e:
            print(e)
    
    def green_mask(self, cv_image):
        """
        masks out elements of the image that have a green value below 70.
        returns output and mask
        """
        #boundaries checking for green values between 70 and 255
        lower_green = np.array([0,70,0],dtype='uint8')
        upper_green = np.array([255,255,255],dtype='uint8')

        mask = cv2.inRange(cv_image,lower_green,upper_green)
        output = cv2.bitwise_and(cv_image, cv_image, mask = mask)

        return output,mask
    

if __name__ == '__main__':
    if(len(sys.argv)>1):
        robot_name = sys.argv[1]
        print(robot_name)
    else:
        robot_name = "thorvald_001"

    ic = image_converter()
    rospy.init_node('vision_node', anonymous=True)
    rospy.spin()
