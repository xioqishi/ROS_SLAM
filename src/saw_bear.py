#!/usr/bin/env python


#This file to control to ground robot to search the bear around the red car

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



def ir_pub():

    ser = serial.Serial('/dev/ttyUSB0',baudrate=115200)

    # Read and flush first line...
    ser.flushInput()
    ser.readline()
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
                        distances.append(int(split_data[0]))
                else:
                    print ("Invalid data: %s" % sensor_data)
            #print (distances[0])

        else:
            print ("Invalid data: %s" % sensor_data)
  
    return distances

def add_two_ints_client(x, y):
    rospy.wait_for_service('add_two_ints')
    try:
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        resp1 = add_two_ints(x, y)
        return resp1.sum
    except rospy.ServiceException, e:
        pass
        #print "Service call failed: %s"%e


def movebase_client(x,y,z,w):

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "base_link"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y  
    goal.target_pose.pose.orientation.w = w
    goal.target_pose.pose.orientation.z = z
    goal.target_pose.pose.orientation.x = 0   
    goal.target_pose.pose.orientation.y = 0  
    #z: -0.38
    #w: 0.92

    client.send_goal(goal)
    
    client.wait_for_result()
    #print(actionlib.SimpleGoalState.DONE)
    #return client.get_result()
    #print(client.wait_for_result())
    state = client.get_state()
    return state


def saw_bear_callback(string):
    if string == "saw_bear":
        global saw_bear
        saw_bear = True
    else:
        pass

def stop_car():
    cancle_pub = rospy.Publisher("move_base/cancel", GoalID, queue_size= 1)
    ttt = GoalID()
    cancle_pub.publish(ttt)




def cw():
    circle_twist = Twist()
    circle_twist.linear.x = 0
    circle_twist.linear.y = 0
    circle_twist.linear.z = 0
    circle_twist.angular.x = 0
    circle_twist.angular.y = 0
    circle_twist.angular.z = 1

    start_time = rospy.Time.now()
    end_time = start_time + rospy.Duration(1.57)
    while (rospy.Time.now() < end_time):
        circle_pub.publish(circle_twist)
        d = rospy.Duration(0.05)
        rospy.sleep(d)
    return 1


def ccw():
    circle_twist_ccw = Twist()
    circle_twist_ccw.linear.x = 0
    circle_twist_ccw.linear.y = 0
    circle_twist_ccw.linear.z = 0
    circle_twist_ccw.angular.x = 0
    circle_twist_ccw.angular.y = 0
    circle_twist_ccw.angular.z = -1.0

    start_time = rospy.Time.now()
    end_time = start_time + rospy.Duration(1.57)
    while (rospy.Time.now() < end_time):
        circle_pub.publish(circle_twist_ccw)
        d = rospy.Duration(0.05)
        rospy.sleep(d)
    return 1



def go_to_four(tttt):
    #while not rospy.is_shutdown():
    if tttt.data == "saw_car":
        print('shou dao che xin hao')
        #global red_car_dis
        
        stop_car()
        #time.sleep(2)
        #x_all = ir_pub()
        
        #x = x_all[0] 
        x = 800
        red_car_dis = ((float(add_two_ints_client(x, 0)) / 1000))
        #red_car_dis = 0.3
        print(red_car_dis)

        #red_car_dis = rospy.Subscriber('/IR_top', Range, red_car_point)
        #red_car_dis = 0.3
        #global saw_bear
        #saw_bear = False
        #rospy.Subscriber('back',String, saw_bear_callback)
        
        circle_twist = Twist()
        circle_twist.linear.x = 0
        circle_twist.linear.y = 0
        circle_twist.linear.z = 0
        circle_twist.angular.x = 0
        circle_twist.angular.y = 0
        circle_twist.angular.z = 0.8

        circle_twist_ccw = Twist()
        circle_twist_ccw.linear.x = 0
        circle_twist_ccw.linear.y = 0
        circle_twist_ccw.linear.z = 0
        circle_twist_ccw.angular.x = 0
        circle_twist_ccw.angular.y = 0
        circle_twist_ccw.angular.z = -0.8



        zero_twist = Twist()
        zero_twist.linear.x = 0.06
        zero_twist.linear.y = 0
        zero_twist.linear.z = 0
        zero_twist.angular.x = 0
        zero_twist.angular.y = 0
        zero_twist.angular.z = 0.08

        
        print('go to 1st point')
        #a1 = movebase_client((red_car_dis - 0.2), 0)
        a1 = 3
        print(a1)
        print('finish 1 st point')
        #if a1:

        #    circle_pub.publish(circle_twist)
        #    rospy.sleep(5.0)
         #   circle_pub.publish(zero_twist)

        print('circle',a1)

        # if a1 == 3 :
        #     a2 = movebase_client(0, 0.45, 0.71, 0.71)

        # if a2 == 3 :
        #     a3 = movebase_client(0.1, 0.1, -0.71, 0.71)

        # if a3 == 3:
        #     a4 = movebase_client(0.65,0, 0,1 )

        # if a4 == 3:
        #     a5 = movebase_client(0.05,0.05,-0.71,0.71)

        # if a5 ==3:
        #     a6 = movebase_client(-0.1,0,0.71,0.71)

        # if a6 == 3:
        #     a7 =  movebase_client(0.5,0,0,1)

        # if a7 == 3:
        #     a8 = movebase_client(0.05,0.05,-0.71,0.71)

        # if a8 == 3:
        #     a9 = movebase_client(0.65,0,0,1)
            


#####
        if a1 ==3:
            circle_pub.publish(zero_twist)
        ########
           
        if a1 == 3:
            #a2_1 = movebase_client(0,  0, 0.71,0.71)
            
            a2_1 = cw()

        if a2_1 == 1:    
            a2 = movebase_client(0.6,  0, 0, 1)
            #circle_pub.publish(circle_twist)
            #time.sleep(5)

            #movebase_client(red_car_position_x + 0.2, red_car_position_y + 0.2)
            #circle_pub.publish(circle_twist)
            print('di_2_ge_dian')
            #time.sleep(5)
        #
        if a2 == 3:
            a3_1 = ccw()


        if a3_1 == 1:
            a3 = movebase_client(1.0, 0,0,1)

        if a3 == 3:
            #a4 = movebase_client(0,  0, -0.71, 0.71)
            a4 = ccw()

        if a4 == 1:
            a5 = movebase_client(1.5,0,0,1)

        if a5 ==3 :
            a6 = ccw()
        
        if a6 == 1:
            a7 = movebase_client(1.0,0,0,1)


        if a7 == 3:
            a8 = ccw()
        
        if a8 == 1:
            movebase_client(0,0,0,1)

        
        
        

        

        #if a4 == 3:
         #   a5 = movebase_client(0.2,  0.5)
        
        #
       # if a3 == 3:
        #    start_time = rospy.Time.now()
         #   end_time = start_time + rospy.Duration(1.97)
          #  while (start_time < end_time):
           #     circle_pub.publish(circle_twist)
            #    d = rospy.Duration(0.05)
             #   rospy.sleep(d)
              #  a4 = 3

        #if a4 == 3:

             
        
       # if a3:
       #     movebase_client(-0.6,  -0.6)
            #movebase_client(red_car_position_x + 0.2, red_car_position_y - 0.2)
            #print('di_4_ge_dian')
            #circle_pub.publish(circle_twist)
            #time.sleep(5)






def red_car_point(ttt):
    global red_car_dis
    red_car_dis = ttt.range
    return red_car_dis
    #print('tis is %f' % ttt.range )

def qiut_callback(string):
    if string.data == 'saw_bear':
        stop_car()
        rospy.signal_shutdown('back_Callback')


if __name__ == '__main__':
    try:
        global red_car_dis
        once_flag = False
        rospy.init_node('movebaseclient_py')
        rospy.Subscriber('/landrobot/object_found', String, go_to_four)
        rospy.Subscriber('/landrobot/object_found',String,qiut_callback)
        circle_pub = rospy.Publisher('cmd_vel', Twist, queue_size=20)
        #rospy.Subscriber('dennis_move_goal', MoveBaseGoal, CommandCallback)
        rospy.spin()
 


    except rospy.ROSInterruptException:
        pass