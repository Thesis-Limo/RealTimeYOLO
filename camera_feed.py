#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

def image_callback(msg):
    bridge = CvBridge()
    try:
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
    else:
        cv2.imshow("Image Window", cv2_img)
        cv2.waitKey(3)

def main():
    rospy.init_node('image_listener')
    image_topic = "/camera/rgb/image_raw"
    rospy.Subscriber(image_topic, Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    main()