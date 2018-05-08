

# import ROS libraries
import rospy
import mavros
from mavros.utils import *
from mavros import setpoint as SP
import mavros.setpoint
import mavros.command
import mavros_msgs.msg
import mavros_msgs.srv
import sys
import signal
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
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
		self.myNode =  rospy.init_node('controller ', anonymous = False) 
		self.rosMsg =  rospy.Subscriber('cmd_msg_to_drone',Float64MultiArray,callback_TMP,queue_size=1) # Subscriber for data to calculate
		

	
	


	    
	def callback_TMP(data):
		print('recieved data')

	
	def callback_vision(topic):
	
	
	
	
	
	def get_drone_position(ros msg) 




	def lower_thr
	


	def increase_thr 



	def pan_left 



	def pan_right 



	def recenter_y 

		if  


if __name__ == '__main__':
    control =  controller()
    
	
