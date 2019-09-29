#!/usr/bin/env python

# this file publish ultrasonic data in ROS


import rospy
from std_msgs.msg import Float32
import time
import random
import RPi.GPIO as GPIO
import numpy as np
import math, logging, time, os, sys
import settings, sensors
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Range
GPIO.setmode(GPIO.BOARD)

#Define pin in raspberry pi
trig = 10 #!!!!!!!!!!
echo = 12 #!!!!!!

#Define I/O 
GPIO.setup(trig, GPIO.OUT)
GPIO.setup(echo, GPIO.IN)



def sensor_distance():
    GPIO.output(trig, GPIO.HIGH)

    time.sleep(0.00001)
    GPIO.output(trig,GPIO.LOW)

    start_time = time.time()
    stop_time = time.time()

    while GPIO.input(echo) == 0:
        start_time = time.time()
    
    while GPIO.input(echo) == 1:
        stop_time = time.time()
        time_duration = stop_time - start_time
        #print(time_duration)
        if(time_duration)>0.5:
            GPIO.output(trig, GPIO.HIGH)
            time.sleep(0.00001)
            GPIO.output(trig,GPIO.LOW)
            break

    distance = time_duration * 17150

    return distance





def distance_sonic():
    pub = rospy.Publisher('IR_top',Range, queue_size=20)
    rospy.init_node('qqqqq', anonymous = True)
    rate = rospy.Rate(10)
    r = Range()
    r.header.frame_id = '/IR_top'
    r.field_of_view =  0.26
    r.max_range = 4.5
    r.min_range = 0.05
    print('IR start')
    while not rospy.is_shutdown():
        r.header.stamp = rospy.Time.now()
        r.range = sensor_distance()
        pub.publish(r)
        rate.sleep()


if __name__ == "__main__":
    try:
        distance_sonic()
    except rospy.ROSInterruptException:
        pass

