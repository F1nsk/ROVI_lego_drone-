#!/usr/bin/env python
# import ROS libraries
import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Vector3
import math
import time
from pid_controller.pid import PID
from std_msgs.msg import String

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
        self.rot_pitch = 0 # ratation around x axsis
        self.distance = 0
        self.valid = False
        self.yImageOffset = 0 #240
        self.xImageOffset = 0 #320
        self.pid_throttle = PID(0.04, 0.0015, 0.001)
        self.pid_roll = PID(0.04, 0.001, 0.0)
        self.pid_yaw = PID(0.00001, 0.0001, 0.0)
        #self.pid_roll = PID(0.00, 0.0000, 0.0)
        self.pid_pitch = PID(0.02, 0.0, 0.05)
        
        self.pidDist = PID(0.006, 0.0001, 0) 
        
        self.pidRot = PID(0.006,0.001, 0)

        self.pid_p_val = 0.0
        self.pid_i_val = 0.0
        self.pid_d_val = 0.0

        self.rosMsg = rospy.Subscriber('ArUco/data_array', Float64MultiArray, self.callback_TMP, queue_size=1) # Subscriber for data to calculate
        self.pidMsg = rospy.Subscriber('PID_controls', String, self.callback_PID, queue_size=1) # Subscriber for data to calculate




#<<<<<<< HEAD
    def init_drone_parameters(self):  # initiates the values, we are using raw ppm values 

        self.thrVal = 500 #lowest  values we can send
        self.rollVal = 520 # middel point
        self.pitchVal = 550 #dito
        self.yawVal =  511 #dito 
#=======
    """
    def init_drone_parameters(self):  # needs values
        print('init')
        self.thrVal = 200
        self.rollVal = 500
        self.pitchVal = 500
        self.yawVal =  370
>>>>>>> e0d5b6f46ca9b5dd3611b22c95c490cbb7a4bc21
    """

    def callback_PID(self, data):

        if data.data == "q":
            self.pid_p_val -= 0.01
        if data.data == "w":
            self.pid_p_val += 0.01
        if data.data == "a":
            self.pid_i_val -= 0.001
        if data.data == "s":
            self.pid_i_val += 0.001
        if data.data == "z":
            self.pid_d_val -= 0.01
        if data.data == "x":
            self.pid_d_val += 0.01

        self.pid_roll = PID(self.pid_p_val, self.pid_i_val, self.pid_d_val)
        print("PID: ", self.pid_p_val, " ", self.pid_i_val, " ", self.pid_d_val)


    def callback_TMP(self, data):
        #print('recieved data')
        self.recenter_x()
        self.recenter_y()
        self.recenter_z()
        
        #self.rotate_drone()
        
        #self.change_dist_to_marker()

        self.catcher()

        self.dummy = data.data

        self.xPos = self.dummy[0]
        self.xPosCentered = self.xPos - self.xImageOffset
        self.yPos = self.dummy[1]
        self.yPosCentered = self.yPos - self.yImageOffset

        self.rotOne = self.dummy[2]
        self.rotTwo = self.dummy[3]
        self.rot_pitch = self.dummy[4]
        self.distance = self.dummy[5]
        self.valid = self.dummy[6]

        if self.rot_pitch < 0:
            self.rot_pitch = self.rot_pitch + 180
        else:
            self.rot_pitch = self.rot_pitch - 180


        print('xPos', self.xPosCentered)
        print('yPos', self.yPosCentered)
        print('rot_pitch:', self.rot_pitch)
        print("distance",self.distance)

        #self.thrVal = 800

        print("Thr : Yaw : Roll : \tPitch")
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
        self.yawVal = 370


    def recenter_y(self):
        if  self.yPosCentered < -5:
            #print('below 0')
            self.thrVal -= self.pid_throttle(self.yPosCentered)
        if 	self.yPosCentered > 5:
            #print('above 0')
            self.thrVal -= self.pid_throttle(self.yPosCentered)
       

    def recenter_x(self):
        if self.xPosCentered >= 3 or self.xPosCentered <= -3:
            if self.xPosCentered < -3:
                print('\troll right')
                self.rollVal += self.pid_roll(self.xPosCentered)
                self.yawVal += self.pid_yaw(self.xPosCentered)
            if self.xPosCentered > 3:
                print('\troll left ')
                self.rollVal += self.pid_roll(self.xPosCentered)
                self.yawVal += self.pid_yaw(self.xPosCentered)

    def recenter_z(self):
        if self.distance >= 45 or self.distance <= 40:
            if self.distance < 40:
                print('\tpitch backward')
                self.pitchVal += self.pid_pitch(self.distance-40)
            if self.distance > 45:
                print('\tpitch forward')
                self.pitchVal += self.pid_pitch(self.distance-45)


    def rotate_drone(self):

        if self.rotTwo > 0:
            self.yawVal += self.pidRot(self.rotOne)
            self.rollVal -= self.pidRot(self.rotOne)
        if self.rotTwo < 0:
            self.yawVal += self.pidRot(self.rotOne)
            self.rollVal -= self.pidRot(self.rotOne)
        

    def change_dist_to_marker(self):

        if self.rotOne > 0:
            self.pitchVal += self.pidDist(self.rotOne)
        if self.rotOne < 0:
            self.pitchVal += self.pidDist(self.rotOne)
        
    def catcher(self):
        if self.rollVal < 450:
            self.rollVal = 450
        if self.rollVal > 560:
            self.rollVal = 560

        if self.yawVal < 450:
            self.yawVal = 450
        if self.yawVal > 560:
            self.yawVal = 560

        if self.thrVal < 0:
            self.thrVal = 0
        if self.thrVal > 1100:
            self.thrVal = 1100

        if self.pitchVal < 450:
            self.pitchVal = 450
        if self.pitchVal > 560:
            self.pitchVal = 560




if __name__ == '__main__':
    rospy.init_node('control_node', anonymous = False)
    control =  Controller()
    control.init_drone_parameters()
    print("test")

    try:
        rospy.spin()

    except KeyboardInterrupt:
        print("Shutting down")



