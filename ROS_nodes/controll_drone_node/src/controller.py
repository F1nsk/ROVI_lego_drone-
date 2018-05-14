

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
		self.xPosCentered # centered to be in the center of the picture 
		self.yPosCentered # dito 
		self.rotOne # rotation around z axsis 
		self.rotTwo # ratation around y axsis 
		self.valid
		self.pidVer = PID(0.006, .0001, 0)
		self.pidHor = PID(0.004, 0.01, 0)
  
		
		
		self.rosMsg =  rospy.Subscriber('cmd_msg_to_drone',Float64MultiArray,callback_TMP,queue_size=1) # Subscriber for data to calculate
		

	
	
	def init_drone_parameters(self):  # needs values 
		
		thrVal = 770
		rollVal = 500
		pitchVal = 500
		yawVal =  370

	
		
	    
	def callback_TMP(self.data):
		print('recieved data')
		self.recenter_x()
		self.recenter_y()
		self.rotate_drone()
		self.change_dist_to_marker()

		dummy = data.data
		
		xPos = dummy[0]
		xPosCentered = xPos - 150 
		yPos = dummy[1]
		yPosCentered = yPos - 150
		
		rotOne = dummy[2]
		rotTwo = dummy[3]
		valid   = dummy[4]
		
		
		array = [self.thrVal,self.yawVal,self.rollVal,self.pitchVal]
    	my_array_for_publishing = Float32MultiArray(data=array)
    	pub(my_array_for_publishing)

	def pup(array):
		img_pub = rospy.Publisher("cmd_msg_to_serial", Float32MultiArray,queue_size=1)
    	img_pub.publish(data)

	def hover(self):
		
		thrVal = 770
		rollVal = 500
		pitchVal = 500
		yawVal =  370


	

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
		
		if rotTwo > 0 
			yawVal  += pid(rotOne) 
			rollVal -= pid(rotOne) 
		if rotTwo < 0 
			yawVal += pid(rotOne) 
			rollVal -= pid(rotOne) 
		else hover()
			
		
		
	
	def change_dist_to_marker(self):
		
		if rotOne > 0 
			pitchVal += pid(rotOne)
		if rotone < 0 
			pitchVal += pid(rotOne)
		else 
			hover() 
			
		
					


if __name__ == '__main__':
	rospy.init_node('controller ', anonymous = False) 
    control =  controller()
	self.init_drone_parameters()

    
	
