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
pathToCam = "/dev/video1"

def main():
    rospy.init_node('Image_capture',anonymous=False)
    tmp = "/home/tgj/catkin_ws/src/ROV_lego_drove-/movies/first.mov"
    #tmp2 = cv2.imread("Selection_002.png")
    cam = cv2.VideoCapture(pathToCam)
    #cam = cv2.VideoCapture(0)
    ret , frame = cam.read()
    cv2.imshow('feed',tmp)
    #frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    pub(frame_gray)

def pub(img):
    bridge = CvBridge()
    msg = bridge.cv2_to_imgmsg(img,encoding="8UC1")
    img_pub = rospy.Publisher('camera/raw',Image,queue_size=1)
    img_pub.publish(msg)

if __name__ == '__main__':
    print('Starting node')
    main()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
