#!/usr/bin/env python
# import ROS libraries
import rospy


# import ROS msg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Int32MultiArray
# Import other libraries
import cv2
import numpy

# Initialize static content
bridge = CvBridge()

def main():
    rospy.init_node('Detect_ArUco',anonymous=False)
    img_sub = rospy.Subscriber('camera/raw',Image,callback,queue_size=1)

"""
Insert your image detection code in this function Anders, and publish in a Int32MultiArry type
"""
def callback(img):
    print('recieved data')
    # pub( my_array_for _publishing)

def pub(data):
    aruco_pub = rospy.Publisher('ArUco/data_array', Int32MultiArray, queue_size=1)
    aruco_pub.publish(data)

if __name__ == '__main__':
    print("Starting node")
    main()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('shutting down')




