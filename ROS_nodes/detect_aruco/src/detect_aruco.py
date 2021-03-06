#!/usr/bin/env python
# import ROS libraries
import rospy


# import ROS msg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float64MultiArray
# Import other libraries
import cv2
import numpy as np

import cv2.aruco as aruco
import yaml
import math

# Initialize static content
bridge = CvBridge()


# https://www.learnopencv.com/rotation-matrix-to-euler-angles/
def rotationMatrixToEulerAngles(R):

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])

def main():
    rospy.init_node('Detect_ArUco',anonymous=False)
    img_sub = rospy.Subscriber('camera/raw',Image,callback,queue_size=1)

"""
Publish in a Int32MultiArray type
"""
def callback(img):
    print('Recieved data')

    with open("calibration2.yaml") as f:
        loadeddict = yaml.load(f)
        camera_matrix = np.asarray(loadeddict.get("camera_matrix"))
        dist_coeffs = np.asarray(loadeddict.get("dist_coeff"))

    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters_create()

    img = bridge.imgmsg_to_cv2(img)

    corners, ids, rejectedImgPoints = aruco.detectMarkers(img, aruco_dict, parameters=parameters)

    # img = aruco.drawDetectedMarkers(img, corners)

    accepted = 0
    if ids != None:
        accepted = 1
        rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 20, camera_matrix, dist_coeffs)

        for i in range (len(rvec)):
            print("Found marker!")
            showImg = aruco.drawAxis(img, camera_matrix, dist_coeffs, rvec[i], tvec[i], 10)
            cv2.imshow("Hello my little friends", showImg)
            cv2.waitKey(1)


            r = cv2.Rodrigues(rvec[i])
            angles = rotationMatrixToEulerAngles(r[0])
            angles = angles / 3.14 * 180

            roll = angles[2]
            yaw = angles[1]
            pitch = angles[0]
            #mid = (corners[0][0][0] + corners[0][0][2]) / 2
            xDist = (tvec[0][i][0])
            yDist = (tvec[0][i][1])
            dist = tvec[0][i][2]


            mydata = [xDist, yDist, yaw, roll, pitch, dist, accepted]
            myPubArray = Float64MultiArray(data=mydata)
            pub(myPubArray)


def pub(data):
    aruco_pub = rospy.Publisher('ArUco/data_array', Float64MultiArray, queue_size=1)
    aruco_pub.publish(data)


if __name__ == '__main__':
    print("Starting node")
    main()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('shutting down')
