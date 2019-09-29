#!/usr/bin/env python

#This file controls the ground robot to avoid obstacles

import math
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
import rospy, time
from sensor_msgs.msg import Range
from actionlib_msgs.msg import GoalID
import rospy
import serial
from std_msgs.msg import String
from rospy_tutorials.srv import *

#define Twist message
def p(x,z):
    w = Twist()
    w.linear.x = x
    w.linear.y = 0
    w.linear.z = 0
    w.angular.x = 0
    w.angular.y = 0        
    w.angular.z = z
    return w                
 

#avoid left obstacles
def avoid_left_callback(msg):
    start_time = rospy.Time.now()
    end_time = start_time + rospy.Duration(0.3)
    if msg.range < 0.3:
        while (rospy.Time.now() < end_time):
                a1 = p(0.08,-0.3)
        
                avoid_pub.publish(a1)
                print("avoid left")
                d = rospy.Duration(0.02)
                rospy.sleep(d)
        while (rospy.Time.now() < (end_time + rospy.Duration(0.3))):
                a2 = p(0.08,0)
                avoid_pub.publish(a2)
                rospy.sleep(d)


# def ooo(x,z):
#         a1 = p(x,z)
#         avoid_pub.publish(a1)

#avoid right obstalces
def avoid_right_callback(msg):
    start_time = rospy.Time.now()
    end_time = start_time + rospy.Duration(0.3)        
    if msg.range < 0.3:
        while (rospy.Time.now() < end_time):
                a1 = p(0.10,0.5)
                avoid_pub.publish(a1)
                d = rospy.Duration(0.02)
                rospy.sleep(d)
        #time.sleep(0.3)

#avoid the obstacles infront of the gournd robot
def avoid_top_callback(msg):
    if msg.range < 0.33:
        a1 = p(0.10,0.5)
        avoid_pub.publish(a1)
        #time.sleep(0.3)



if __name__ == '__main__':
    try:
        rospy.init_node('avoid_node')
        rospy.Subscriber('IR_left',Range ,avoid_left_callback)
        rospy.Subscriber('IR_right',Range ,avoid_right_callback)                        
        avoid_pub = rospy.Publisher('avoid_vel', Twist, queue_size=10)
        rospy.spin()
 


    except rospy.ROSInterruptException:
        pass 