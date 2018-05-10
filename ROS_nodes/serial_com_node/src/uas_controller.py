#!/usr/bin/env python
# import ROS libraries
import rospy
import sys
import signal

# Ros msg libraries
from std_msgs import Float64MultiArray



import time
import serial

class uas_serial_controller:

    def __init__(self):
        self.ser = serial.Serial(
            port='/dev/ttyUSB0',
            baudrate=115200,
            parity=serial.PARITY_ODD,
            stopbits=serial.STOPBITS_TWO,
            bytesize=serial.SEVENBITS
        )
        rospy.Subscriber('cmd_msg_to_serial',Float64MultiArray,callback,queue_size=1)

    def callback(self,data):
        print('recieved msg')
        self.initialize_serial()
        self.make_string(data)
        self.pub(msg)


    def make_string(self, array):

        myStr = str
        intList = []
        for x in len(array):
            intList[x] = int(array[x])

        """
        delim = ':'
        ###### Sends the characters to the arduino
        # prepare one string for sending
        myStr = str(self.throttle)
        myStr += delim
        myStr += str(self.roll)
        myStr += delim
        myStr += str(self.elevator)
        myStr += delim
        myStr += str(self.rudder)

        print(myStr)
        # print('type ' , type(input))
        """

        self.ser.write(myStr)
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


    def pub(self):
        rospy.Publisher() ## TODO





if __name__ == '__main__':
    usc = uas_serial_controller()
    
