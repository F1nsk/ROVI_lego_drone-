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
VERBOSE = True
#tmp ='/home/finsk/catkin_ws/src/ROVI_lego_drone-/movies/first.mov'
#cap = cv2.VideoCapture(tmp)

def main():
    rospy.init_node('Image_capture',anonymous=False)
    #cam = cv2.VideoCapture(tmp)
    cam = cv2.VideoCapture(pathToCam)
    
    while True :
        ret, frame = cam.read()
        
        #frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        pub(frame)
        height, width = frame.shape[:2]
        print('(H,W)',height,width)
        if VERBOSE == True:    
          #  cv2.imshow('feed',frame)
            cv2.waitKey(1)
            
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break 
        
    cam.release()
    cv2.destroyAllWindows()
#>>>>>>> master

def pub(img):
    bridge = CvBridge()
    msg = bridge.cv2_to_imgmsg(img,encoding="8UC3")
    img_pub = rospy.Publisher('camera/raw',Image,queue_size=1)
    img_pub.publish(msg)

if __name__ == '__main__':
    print('Starting node')
    main()
try:
    rospy.spin()
except KeyboardInterrupt:
    print("Shutting down")

