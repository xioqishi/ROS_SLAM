#!/usr/bin/python

#This file can use keyboard to control the ground robot

import rospy
from std_msgs.msg import String
import time
from geometry_msgs.msg import Twist
import readchar


tem_control = Twist()
tem_control.linear.y = 0
tem_control.linear.z = 0
tem_control.angular.x = 0
tem_control.angular.y = 0
def send_go():
    a = readchar.readchar()


    if (a =='w' ):
 
        tem_control.linear.x = 0.4
        tem_control.angular.z = 0

    
    elif(a =='s'):

        tem_control.linear.x = -0.2
        tem_control.angular.z = 0

    elif(a =='d'):

        tem_control.angular.z = -0.4
        tem_control.linear.x = 0

    elif(a =='a'):

        tem_control.angular.z = 0.4
        tem_control.linear.x = 0

    elif (a == 'q'):
        tem_control.linear.x = 0.0
        tem_control.angular.z = 0.0

        
    return tem_control



def distance_sonic():
    pub = rospy.Publisher('twist_mux/cmd_vel',Twist, queue_size=10)
    rospy.init_node('key_board_control', anonymous = True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        a = send_go()
        pub.publish(a)
        rate.sleep()


if __name__ == "__main__":
    try:
        distance_sonic()
    except rospy.ROSInterruptException:
        rospy.loginfo("finished")

