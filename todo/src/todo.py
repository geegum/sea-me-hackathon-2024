#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, Joy
from std_msgs.msg import Float32
import cv2

dist = 0.0
camImageCopy_ = None

def ImageSubCallback(msg):
    global camImageCopy_
    bridge = CvBridge()
    try:
        cam_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
    
    if cam_image is not None:
        camImageCopy_ = cam_image.copy()

def DistanceSubCallback(dist_msg):
    global dist
    dist = dist_msg.data

def JoySubCallback(joy_msg):
    pass  # 아직 구현되지 않음

def main():
    rospy.init_node('todo', anonymous=True)
    
    rospy.Subscriber("/usb_cam/image_raw", Image, ImageSubCallback)
    rospy.Subscriber("/distance", Float32, DistanceSubCallback)
    rospy.Subscriber("/joy", Joy, JoySubCallback)
    
    steering_publisher = rospy.Publisher('/Steering', Float32, queue_size=1)
    throttle_publisher = rospy.Publisher('/Throttle', Float32, queue_size=1)
    
    rate = rospy.Rate(10)  # 10Hz
    
    while not rospy.is_shutdown():
        cv2.namedWindow("image")
        if camImageCopy_ is not None:
            cv2.imshow("image", camImageCopy_)
            cv2.waitKey(1)
        
        rospy.loginfo("dist: %f", dist)
        
        steering_msg = Float32()
        throttle_msg = Float32()
        steering_msg.data = 0.0
        throttle_msg.data = 0.0
        
        steering_publisher.publish(steering_msg)
        throttle_publisher.publish(throttle_msg)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
