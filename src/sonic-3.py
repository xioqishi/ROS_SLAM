#!/usr/bin/env python

#this file test for the untrasonce sensors


import rospy
from std_msgs.msg import Float32
import time
import random
import RPi.GPIO as GPIO

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
        if(time_duration)>2:
            GPIO.output(trig, GPIO.HIGH)
            time.sleep(0.00001)
            GPIO.output(trig,GPIO.LOW)
            break

    distance = time_duration * 17150

    return distance





def distance_sonic():
    pub = rospy.Publisher('sonic_distance',Float32, queue_size=20)
    rospy.init_node('visual', anonymous = True)
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        distance = sensor_distance()
        pub.publish(distance)
        rate.sleep()


if __name__ == "__main__":
    try:
        distance_sonic()
    except rospy.ROSInterruptException:
        pass

