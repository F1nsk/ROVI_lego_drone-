

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
		self.rotOne
		self.Two
		self.valid
		
		self.rosMsg =  rospy.Subscriber('cmd_msg_to_drone',Float64MultiArray,callback_TMP,queue_size=1) # Subscriber for data to calculate
		

	
	


	    
	def callback_TMP(data):
		print('recieved data')
		
		dummy = data.data
		
		xPos = dummy[0]
		yPos = dummy[1]
		rotOne = dummy[2]
		rotTwo = dummy[3]
		valid   = dummy[4]
		
		


	def postion() 
		data 
	
	def callback_vision(topic):
	
	
	
	
	
	def get_drone_position(ros msg) 




	def lower_thr
	


	def increase_thr 



	def pan_left 



	def pan_right 



	def recenter_y 

		if  


if __name__ == '__main__':
	rospy.init_node('controller ', anonymous = False) 
    control =  controller()
    
	
