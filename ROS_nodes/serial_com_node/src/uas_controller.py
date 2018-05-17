#!/usr/bin/env python
# import ROS libraries
import rospy
import sys
import signal

# Ros msg libraries
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String



import time
import serial

class uas_serial_controller:

    def __init__(self):
        self.myStr = str
        self.ser = serial.Serial(
            port='/dev/ttyUSB1',
            baudrate=115200,
            parity=serial.PARITY_ODD,
            stopbits=serial.STOPBITS_TWO,
            bytesize=serial.SEVENBITS
        )
        self.initialize_serial()
        self.arm_seq = "0000110000000000"
        self.arm_flag = False

        rospy.init_node('Serial_com_node',anonymous=False)
        rospy.Subscriber('cmd_msg_to_serial',Float64MultiArray,self.callback,queue_size=1)
	    
    def callback(self,data):
        #print('recieved msg')
        myData = data.data
        if self.arm_flag == False:
            print('False flag')
            self.arm_device()
        
        if self.arm_flag == True:
            self.initialize_serial()
            self.make_string(myData)
            self.pub(myData)

    def arm_device(self):
        m_input = raw_input(">> ")
        if m_input == ("arm"):
            self.initialize_serial()
            self.writeOut(self.arm_seq)
            time.sleep(2)
            self.arm_flag=True
        
        

    def make_string(self, array):
        #myStr = str
        intList = [0,0,0,0,0]
        end = len(array)
        for x in range(end):
            tmp = array[x]
            intList[x]=int(tmp)
        #print('******************\n')
        print('in: ',intList)
        #print('******************\n')
        throttle    = str(intList[0])
        rudder      = str(intList[1])
        roll        = str(intList[2])
        pitch       = str(intList[3])
        delim       = ':'

        ##### Sends the characters to the arduino
        # prepare one string for sending
        while len(throttle) < 4:
            throttle = "0"+throttle
        while len(rudder) < 4:
            rudder = "0"+rudder
        while len(roll) < 4:
            roll = "0"+roll
        while len(pitch) < 4:
            pitch = "0"+pitch
        self.myStr = throttle
        #self.myStr += delim
        self.myStr += rudder
        #self.myStr += delim
        self.myStr += roll
        #self.myStr += delim
        self.myStr += pitch
        print('str ',self.myStr)
        self.ser.write(self.myStr)
        
        out = ''

        time.sleep(0.2)  # Give the client some time to repons

        while self.ser.inWaiting() > 0:
            out += self.ser.read(1)

        # Print put what was entered in the terminal
        if out != '':
            print(">>" + out)
	
    def writeOut(self,string):
        self.ser.write(string)

    def initialize_serial(self):
        self.ser.isOpen()
        input=1


    def pub(self,msg):
        myPub = rospy.Publisher('Echo_cmds',String,queue_size=1)
        myPub.publish(msg)

    def testConnection(self):
        out = ''
        m_input = raw_input(">> ")
        if m_input == ("arm"):
            
            usc.initialize_serial()
            
            arm_seq = "11401860000000000"
            
            self.ser.write(arm_seq)
            print(arm_seq)
               
        usc.initialize_serial()
        
        self.ser.write(m_input)
            
            
        time.sleep(0.2)  # Give the client some time to repons
            
        while self.ser.inWaiting() > 0:
            out += self.ser.read()

        # Print put what was entered in the terminal
        #if out != '':
        print(">>" + out)
            



if __name__ == '__main__':
    #arm_seq = "0:1100:0:0"
    usc = uas_serial_controller()
    #usc.arm_device()
    
    #while True:
     #   usc.testConnection()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    
