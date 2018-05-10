#!/usr/bin/env python
# import ROS libraries
import rospy
import sys
import signal

# Ros msg libraries
from std_msgs.msg import Float64MultiArray



import time
import serial

class uas_serial_controller:

    def __init__(self):
        self.myStr = str
        self.ser = serial.Serial(
            #port='/dev/ttyUSB0',
            baudrate=115200,
            parity=serial.PARITY_ODD,
            stopbits=serial.STOPBITS_TWO,
            bytesize=serial.SEVENBITS
        )
        rospy.init_node('Serial_com_node',anonymous=False)
        rospy.Subscriber('cmd_msg_to_serial',Float64MultiArray,self.callback,queue_size=1)

    def callback(self,data):
        print('recieved msg')
        myData = data.data
        self.initialize_serial()
        self.make_string(myData)
        self.pub(myData)


    def make_string(self, array):

        #myStr = str
        intList = [0,0,0,0,0]
        end = len(array)
        for x in range(end):
            tmp = array[x]
            intList[x]=int(tmp)
        print(intList)
        throttle    = str(intList[0])
        rudder      = str(intList[1])
        roll        = str(intList[2])
        pitch       = str(intList[3])
        delim       = ':'

        ##### Sends the characters to the arduino
        # prepare one string for sending

        self.myStr = throttle
        self.myStr += delim
        self.myStr += rudder
        self.myStr += delim
        self.myStr += roll
        self.myStr += delim
        self.myStr += pitch

        self.ser.write(self.myStr)
        """"""
        out = ''

        time.sleep(1)  # Give the client some time to repons

        while self.ser.inWaiting() > 0:
            out += self.ser.read(1)

        # Print put what was entered in the terminal
        if out != '':
            print(">>" + out)


    def initialize_serial(self):
        self.ser.isOpen()
        input=1


    def pub(self,msg):
        myPub = rospy.Publisher('Echo_cmds',String,queue_size=1)
        myPub.publish(msg)





if __name__ == '__main__':
    usc = uas_serial_controller()
    array = [1.1,2.2,3.3,4.4,5.5]
    usc.make_string(array)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    
