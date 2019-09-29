#!/usr/bin/env python
#This file publishs IR sensors information in Range message 

import numpy as np
import math, logging, time, os, sys
import settings, sensors
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Range


def top_IR():
    distance_top = sensors.get_distance(sensors.read_adc(0),'2y0a21') / 100.0
    #print(distance_top)
    return distance_top

def left_IR():
    distance_left = sensors.get_distance(sensors.read_adc(2),'2y0a21') / 100.0
    #print(distance_top)
    return distance_left

def right_IR():
    distance_right = sensors.get_distance(sensors.read_adc(3),'2y0a21') / 100.0
    return distance_right

def talker():
    pub = rospy.Publisher('/IR_top', Range, queue_size=20)
    left_pub = rospy.Publisher('/IR_left', Range, queue_size=20)
    right_pub = rospy.Publisher('/IR_right', Range, queue_size=20)
    rospy.init_node('IR_node', anonymous=True)
    rate = rospy.Rate(10) # 5hz
    r = Range()
    r.header.frame_id = '/IR_top'
    #r.radiation_type = Range.INFRARED
    r.field_of_view =  0.21
    r.max_range = 0.80
    r.min_range = 0.10

    left = Range()
    left.header.frame_id = '/IR_left'
    #left.radiation_type = Range.INFRARED
    left.field_of_view =  0.21
    left.max_range = 0.80
    left.min_range = 0.10    


    right = Range()
    right.header.frame_id = '/IR_right'
    #right.radiation_type = Range.INFRARED
    right.field_of_view = 0.21
    right.max_range = 0.80
    right.min_range = 0.1

    print('IR start')
    while not rospy.is_shutdown():
        r.header.stamp = rospy.Time.now()
        r.range = top_IR()
        pub.publish(r)
        
        
        left.header.stamp = rospy.Time.now()
        left.range = left_IR()
        left_pub.publish(left)
        
        right.header.stamp = rospy.Time.now()
        right.range = right_IR()
        right_pub.publish(right)
        
        rate.sleep()





if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
