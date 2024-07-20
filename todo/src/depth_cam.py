#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

def depth_image_callback(msg):
    try:
        # Convert the ROS Image message to an OpenCV image
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, "passthrough")

        # Display the depth image using OpenCV
        cv2.imshow("Depth Image", cv_image)
        cv2.waitKey(1)
    except CvBridgeError as e:
        print(e)

def main():
    rospy.init_node('depth_cam_node', anonymous=True)
    rospy.Subscriber('/camera/depth/image_raw', Image, depth_image_callback)
    rospy.spin()

if __name__ == '__main__':
    main()

