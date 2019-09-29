#!/usr/bin/env python


#This file publish laserscan data 

import rospy
from sensor_msgs.msg import LaserScan
import serial
from math import pi
import os,time
from sensor_msgs.msg import Range
ser = serial.Serial('/dev/ttyUSB0',baudrate=115200)


#Collect data from TOF sensors

def ir_pub():
    ser.flushInput()
    ser.readline()
    statuses = []
    distances = []
    #global statuses
    #global distances
    ser_bytes = ser.readline().decode().strip()
    if ser_bytes.startswith('#'):
        print (ser_bytes[1:].strip())
    else: 
        sensor_data = ser_bytes.split(",")
        if len(sensor_data)==16:

            for element in sensor_data:
                split_data = element.split(";")
                if len(split_data) == 2:
                    status = split_data[1]
                    statuses.append(status)
                    if status=='-1':
                        distances.append(-1)
                    else:
                        distances.append(float(split_data[0])/1000)
                else:
                    print ("Invalid data: %s" % sensor_data)
            #print (distances[0])

        else:
            print ("Invalid data: %s" % sensor_data)
  
    return distances

def talker():
    top_pub = rospy.Publisher('/IR_top', Range, queue_size=1)
    left_pub = rospy.Publisher('/IR_left', Range, queue_size=1)
    right_pub = rospy.Publisher('/IR_right', Range, queue_size=1)
    rospy.init_node('IR_node', anonymous=True)
    rate = rospy.Rate(6) # 10hz

    #top_data
    top_range = Range()
    top_range.header.frame_id = '/IR_top'
    top_range.field_of_view =  0.4
    top_range.max_range = 3
    top_range.min_range = 0.003
    print('TOF start')

    #left
    left_range = Range()
    left_range.header.frame_id = '/IR_left'
    left_range.field_of_view =  0.4
    left_range.max_range = 3
    left_range.min_range = 0.005

    #right
    right_range = Range()
    right_range.header.frame_id = '/IR_right'
    right_range.field_of_view =  0.4
    right_range.max_range = 3
    right_range.min_range = 0.003

    time.sleep(3)
    while not rospy.is_shutdown():
        receiced_distance = ir_pub()
        #top
        top_range.header.stamp = rospy.Time.now()
        top_range.range = receiced_distance[0]
        #print('top is %f \n' % top_range.range)
        top_pub.publish(top_range)

        #left
        left_range.header.stamp = rospy.Time.now()
        left_range.range = receiced_distance[12]
       # print('left is %f \n' %left_range.range)
        left_pub.publish(left_range)


        #right
        right_range.header.stamp = rospy.Time.now()
        right_range.range = receiced_distance[4]
        #print('right is %f \n' % right_range.range)
        right_pub.publish(right_range)        


        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
