#!/usr/bin/env python
# import ROS libraries
import rospy
from geometry_msgs.msg import Vector3
from std_msgs.msg import String
import math
import time




def main():
	pid_pub = rospy.Publisher("PID_controls", String, queue_size=1)
	pid_input = raw_input("Increase PID >> ")
	if pid_input == "q":
		pid_pub.publish(pid_input)
	if pid_input == "w":
		pid_pub.publish(pid_input)
	if pid_input == "a":
		pid_pub.publish(pid_input)
	if pid_input == "s":
		pid_pub.publish(pid_input)
	if pid_input == "z":
		pid_pub.publish(pid_input)
	if pid_input == "x":
		pid_pub.publish(pid_input)

if __name__ == '__main__':
	print('Init Node')
	rospy.init_node('PID_controller', anonymous=False)
	while True:
		main()

