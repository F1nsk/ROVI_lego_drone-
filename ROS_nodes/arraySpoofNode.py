#!/usr/bin/env python
# import ROS libraries
import rospy
import sys
import signal

# Ros msg libraries
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from controll_drone_node.msg import MyArray




def main():
    pub = rospy.Publisher('cmd_msg_to_serial', MyArray, queue_size=1)
    myPub2 = rospy.Publisher('test', String, queue_size=1)
    rospy.init_node('ArraySpoof',anonymous=False)
    array = [1.1, 2.2, 3.3, 4.4, 5.5]
    rate = rospy.Rate(10)  # 10hz
    message = MyArray
    print('string',type(String))
    print('mine',type(message))
    message.throttle = 0.1
    #message.rudder=0.12
    #message.pitch=2.2
    #message.roll=3.3
    print(message.throttle)

    while not rospy.is_shutdown():

        print(type(message))
        pub.publish(message)
        #myPub2.publish("hello")
        rate.sleep()

    print('published**')

if __name__ == '__main__':
    main()
    print('after main')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

