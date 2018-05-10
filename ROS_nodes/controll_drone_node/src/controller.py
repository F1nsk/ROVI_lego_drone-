

# import ROS libraries
import rospy
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Vector3
import math
import time
from pid_controller.pid import PID



def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


current_pose = SP.PoseStamped()
global new_lower_pos
new_lower_pos = 10000


global pixel_pos
pixel_pos = [0, 0]

UAV_state = mavros_msgs.msg.State()
global last_request
global set_mode



class  controller:
	
	""" Class doc """
	
	def __init__ (self):
		self.thrVal 
		self.rollVal 
		self.pitchVal 
		self.yawVal 
		self.dummy
		self.xPos
		self.yPos
		self.xPosCentered 
		self.yPosCentered 
		self.rotOne
		self.rotTwo
		self.valid
		self.pidVer = PID(0.006, .0001, 0)
		self.pidHor = PID(0.004, 0.01, 0)
  
		
		
		self.rosMsg =  rospy.Subscriber('cmd_msg_to_drone',Float64MultiArray,callback_TMP,queue_size=1) # Subscriber for data to calculate
		

	
	


	    
	def callback_TMP(self.data):
		print('recieved data')
		
		dummy = data.data
		
		xPos = dummy[0]
		xPosCentered = xPos - 150 
		yPos = dummy[1]
		yPosCentered = yPos - 150
		
		rotOne = dummy[2]
		rotTwo = dummy[3]
		valid   = dummy[4]
		
		
		



	
	def callback_vision(self):
	
	
	
	
	
	def get_drone_position(self):  
		
	def hover(self):
		thrVal =   #find value 

	

	def recenter_y(self):  

		if  yPosCentered > 0 
			thrVal -= pid(yPosCentered) 
		if 	yPoscentered < 0 
			thrVal += pid(yPosCentered)
		else 
			hover() 
			
	def recenter_x(self): 
		
		if xPosCentered > 0
			rollVal -= pid(xPosCentered) 
		if xPosCentered < 0 
			rollVal += pid(xPosCentered)  
		else 
			hover() 
			
	def rotate_drone(self):
		
		
		
		
		
			
			


if __name__ == '__main__':
	rospy.init_node('controller ', anonymous = False) 
    control =  controller()
    
	
