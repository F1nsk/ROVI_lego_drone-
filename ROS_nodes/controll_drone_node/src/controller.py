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
        self.yImageOffset = 240
        self.xImageOffset = 320
        self.pidVer = PID(0.0006, 0.001, 0)
        self.pidHor = PID(0.004, 0.0001, 0)
        
        self.pidDist = PID(0.006, 0.0001, 0) 
        
        self.pidRot = PID(0.006,0.001, 0) 



        self.rosMsg =  rospy.Subscriber('ArUco/data_array' ,Float64MultiArray , self.callback_TMP ,queue_size=1) # Subscriber for data to calculate




    def init_drone_parameters(self):  # needs values
        print('init')
        self.thrVal = 200
        self.rollVal = 500
        self.pitchVal = 500
        self.yawVal =  370




    def callback_TMP(self, data):
        #print('recieved data')
        #self.recenter_x()
        self.recenter_y()
        
        #self.rotate_drone()
        
        #self.change_dist_to_marker()

        self.dummy = data.data

        self.xPos = self.dummy[0]
        self.xPosCentered = self.xPos - self.xImageOffset
        self.yPos = self.dummy[1]
        self.yPosCentered = self.yPos - self.yImageOffset

        self.rotOne = self.dummy[2]
        self.rotTwo = self.dummy[3]
        self.valid = self.dummy[5]

        print('xPos', self.xPosCentered)
        print('yPos', self.yPosCentered)
        
        array = [self.thrVal ,self.yawVal ,self.rollVal ,self.pitchVal]
        print(array)
        
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

        if  self.yPosCentered < 0:
            print('below 0')
            self.thrVal += self.pidVer(self.yPosCentered)
        if 	self.yPosCentered > 0:
            print('above 0')
            self.thrVal += self.pidVer(self.yPosCentered)
       

    def recenter_x(self):

        if self.xPosCentered < 0:
            print('below 0')
            self.rollVal += self.pidHor(self.xPosCentered)
        if self.xPosCentered > 0:
            print('above 0')
            self.rollVal += self.pidHor(self.xPosCentered)
        

    def rotate_drone(self):

        if self.rotTwo > 0:
            self.yawVal  += self.pidRot(self.rotOne)
            self.rollVal -= self.pidRot(self.rotOne)
        if self.rotTwo < 0:
            self.yawVal += self.pidRot(self.rotOne)
            self.rollVal -= self.pidRot(self.rotOne)
        

    def change_dist_to_marker(self):

        if self.rotOne > 0:
            self.pitchVal += self.pidDist(self.rotOne)
        if self.rotOne < 0:
            self.pitchVal += self.pidDist(self.rotOne)
        





if __name__ == '__main__':
    rospy.init_node('control_node', anonymous = False)
    control =  Controller()
    control.init_drone_parameters()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")



