#!/usr/bin/env python
# import ROS libraries
import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Vector3
import math
import time
from pid_controller.pid import PID

class  Controller:

    """ Class doc """

    def __init__ (self):
        self.thrVal = 0
        self.rollVal = 0
        self.pitchVal = 0
        self.yawVal = 0
        self.dummy = []
        self.xPos = 0
        self.yPos = 0
        self.xPosCentered = 0 # centered to be in the center of the picture
        self.yPosCentered = 0 # dito
        self.rotOne = 0 # rotation around z axsis
        self.rotTwo = 0 # ratation around y axsis
        self.valid = False
        self.pidVer = PID(0.006, .0001, 0)
        self.pidHor = PID(0.004, 0.01, 0)
        self.pidDist = PID(0.006, .0001, 0) 
        self.pidRot = PID(0.006,0.001, 0) 



        self.rosMsg =  rospy.Subscriber('ArUco/data_array' ,Float64MultiArray , self.callback_TMP ,queue_size=1) # Subscriber for data to calculate




    def init_drone_parameters(self):  # needs values

        self.thrVal = 770
        self.rollVal = 500
        self.pitchVal = 500
        self.yawVal =  370




    def callback_TMP(self, data):
        print('recieved data')
        self.recenter_x()
        self.recenter_y()
        self.rotate_drone()
        self.change_dist_to_marker()

        self.dummy = data.data

        self.xPos = self.dummy[0]
        self.xPosCentered = self.xPos - 150
        self.yPos = self.dummy[1]
        self.yPosCentered = self.yPos - 150

        self.rotOne = self.dummy[2]
        self.rotTwo = self.dummy[3]
        self.valid = self.dummy[4]


        array = [self.thrVal ,self.yawVal ,self.rollVal ,self.pitchVal]
        my_array_for_publishing = Float64MultiArray(data=array)
        self.pub(my_array_for_publishing)

    def pub(self, array):
        img_pub = rospy.Publisher("cmd_msg_to_serial", Float64MultiArray ,queue_size=1)
        img_pub.publish(array)

    def hover(self):
        self.thrVal = 770
        self.rollVal = 500
        self.pitchVal = 500
        self.yawVal =  370


    def recenter_y(self):

        if  self.yPosCentered > 0:
            self.thrVal -= self.pidVer(self.yPosCentered)
        if 	self.yPosCentered < 0:
            self.thrVal += self.pidVer(self.yPosCentered)
        else:
            self.hover()

    def recenter_x(self):

        if self.xPosCentered > 0:
            self.rollVal -= self.pidHor(self.xPosCentered)
        if self.xPosCentered < 0:
            self.rollVal += self.pidHor(self.xPosCentered)
        else:
            self.hover()

    def rotate_drone(self):

        if self.rotTwo > 0:
            self.yawVal  += self.pidRot(self.rotOne)
            self.rollVal -= self.pidRot(self.rotOne)
        if self.rotTwo < 0:
            self.yawVal += self.pidRot(self.rotOne)
            self.rollVal -= self.pidRot(self.rotOne)
        else:
            self.hover()




    def change_dist_to_marker(self):

        if self.rotOne > 0:
            self.pitchVal += self.pidDist(self.rotOne)
        if self.rotOne < 0:
            self.pitchVal += self.pidDist(self.rotOne)
        else:
            self.hover()





if __name__ == '__main__':
    rospy.init_node('control_node', anonymous = False)
    control =  Controller()
    control.init_drone_parameters()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")



