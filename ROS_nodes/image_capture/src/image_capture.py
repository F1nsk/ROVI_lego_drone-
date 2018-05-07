#!/usr/bin/env python
# import ROS libraries
import rospy

# ROS msg libraries
from sensor_msgs.msg import Image

# Other libraries
from cv_bridge import CvBridge
import cv2
import numpy
from matplotlib import pyplot as plt


def main():
    rospy.init_node('Image_capture',anonymous=False)
    tmp = "/home/tgj/catkin_ws/src/ROV_lego_drove-/movies/first.mov"
    cam = cv2.VideoCapture(tmp)
    #cam = cv2.VideoCapture(0)
    ret , frame = cam.read()
    #cv2.imshow('feed',frame)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #pub(gray)

def pub(img):
    img_pub = rospy.Publisher('camera/raw',Image,queue_size=1)
    img_pub.publish(img)

if __name__ == '__main__':
    print('Starting node')
    main()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
