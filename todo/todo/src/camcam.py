#!/usr/bin/env python

# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np

steering_weight = 1.5 
throttle_weight = 0.1 


pub_steering = rospy.Publisher('/Steering', Float32, queue_size=1)
pub_throttle = rospy.Publisher('/Throttle', Float32, queue_size=1)


gijun = 30000
cntgijun = 1

class CameraDisplay:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)

        # Create trackbars for adjusting HSV threshold values
        cv2.namedWindow("Thresholded Image")
        cv2.createTrackbar("Hue Min", "Thresholded Image", 0, 180, self.update_threshold)
        cv2.createTrackbar("Hue Max", "Thresholded Image", 180, 180, self.update_threshold)
        cv2.createTrackbar("Sat Min", "Thresholded Image", 0, 255, self.update_threshold)
        cv2.createTrackbar("Sat Max", "Thresholded Image", 255, 255, self.update_threshold)
        cv2.createTrackbar("Val Min", "Thresholded Image", 0, 255, self.update_threshold)
        cv2.createTrackbar("Val Max", "Thresholded Image", 20, 255, self.update_threshold)

        self.lower_hsv = np.array([9, 44, 185])
        self.upper_hsv = np.array([31, 192, 215])

    def update_threshold(self, _):
        self.lower_hsv = np.array([cv2.getTrackbarPos("Hue Min", "Thresholded Image"),
                                  cv2.getTrackbarPos("Sat Min", "Thresholded Image"),
                                  cv2.getTrackbarPos("Val Min", "Thresholded Image")])
        self.upper_hsv = np.array([cv2.getTrackbarPos("Hue Max", "Thresholded Image"),
                                  cv2.getTrackbarPos("Sat Max", "Thresholded Image"),
                                  cv2.getTrackbarPos("Val Max", "Thresholded Image")])

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("CvBridge Error: {}".format(e))
            return

        # Convert the image to HSV
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Apply the threshold to the HSV image
        thresholded_image = cv2.inRange(hsv_image, self.lower_hsv, self.upper_hsv)
        #rospy.loginfo("thresholded_image: {}".format(thresholded_image))
        #result = cv2.bitwise_and(frame, frame, thresholded_image=thresholded_image)
        # sum
        # Calculate the area of the left and right regions
        h, w = thresholded_image.shape
        wvX = np.arange(0,w) 
        wvY1 = np.arange(0,h) 
        wvY = (wvY1.reshape(480,1))/10
        wvXX = wvX - (w/2)
        tresh = thresholded_image/255
        
        tresh12 = tresh * wvXX
        lineDetect = np.sum(tresh12)
        if lineDetect > 1500000:
            rospy.loginfo("lineDetect: {}".format("right"))
            pub_steering.publish(1)
        elif lineDetect < -1500000:
            rospy.loginfo("lineDetect: {}".format("Left"))
            pub_steering.publish(1)
        else:
            rospy.loginfo("lineDetect: {}".format("gogo"))

	pub_throttle.publish(0.3)
        
        
        rospy.loginfo("lineDetect: {}".format(lineDetect))

        

        # Display the original, HSV, and thresholded images
        cv2.imshow("Camera Image", cv_image)
        cv2.imshow("HSV Image", hsv_image)
        cv2.imshow("Thresholded Image", thresholded_image)
        cv2.waitKey(1)

def main():
    rospy.init_node('camera_display', anonymous=True)
    CameraDisplay()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
	pub_throttle.publish(0.0)
    
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

