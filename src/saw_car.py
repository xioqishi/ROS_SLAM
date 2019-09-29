#!/usr/bin/env python


# This file wait the vector from drone, if it dosen not recevie the message, it will explore the areas

import rospy,time, math
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalID
from std_msgs.msg import String, Bool,Int16, Empty
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

def movebase_client(a,x,y):

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

# 1 means the relative position between 'target position' and 'base_link'
# 2 means the relative position between 'target position' and 'base_link'
    if a == 1:
        p = "base_link"
    elif a == 2:
        p = "map"
        
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = p
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y  
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)
    
    client.wait_for_result()
    state = client.get_state()
    return state

def back_Callback(mag_t):
    global drone_saw_first
    if mag_t.data == "back":
        #state = "back"
        movebase_client(2,0,0)

    # if the camera see the car, the node will close
    elif mag_t.data == "saw_car":
        drone_saw_first = False
        cancle_pub = rospy.Publisher("move_base/cancel", GoalID, queue_size= 1)
        ttt = GoalID()
        cancle_pub.publish(ttt)
        rospy.signal_shutdown('back_Callback')


global drone_saw_first
drone_saw_first = True




# Receive the relative postion from drone and go to that position
def CommandCallback(receive_goal):
    global drone_saw_first
    global drone_saw
    drone_saw = False
    if drone_saw_first == True:
        if receive_goal.linear.y == 0:
            r = -(receive_goal.angular.z)
            x = receive_goal.linear.x * math.cos(r)
            y = receive_goal.linear.x * math.sin(r)
            #client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
            #client.wait_for_server()
            #client.send_goal(receive_goal)
            movebase_client(1,x,y)
            drone_saw = True 
        # elif receive_goal.linear.y == 1:
        #     r = -(receive_goal.angular.z) - 3.14/2
        #     x = -receive_goal.linear.x * math.cos(r)
        #     y = -receive_goal.linear.x * math.sin(r)
        #     movebase_client(2,x,y)
        #     drone_saw = True 
        # elif receive_goal.linear.y == 2:
        #     r = -(receive_goal.angular.z) + 3.14
        #     x = receive_goal.linear.x * math.cos(r)
        #     y = receive_goal.linear.x * math.sin(r)
        #     movebase_client(2,x,y)
        #     drone_saw = True
        # elif receive_goal.linear.y == 3:
        #     r = -(receive_goal.angular.z)
        #     x = receive_goal.linear.x * math.cos(r)
        #     y = receive_goal.linear.x * math.sin(r)
        #     #client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        #     #client.wait_for_server()
        #     #client.send_goal(receive_goal)
        #     movebase_client(2,x,y)
        #     drone_saw = True

        # elif (drone_saw == False) and (rospy.Time.now() > end_time):
        #     movebase_client(1, 1,-0.5)
        #     print('fei ji mei kan dao')                             





if __name__ == '__main__':
    global drone_saw

    try:
        rospy.init_node('movebase_client_py')
        global state
        global end_time
        start_time = rospy.Time.now()
        end_time = start_time + rospy.Duration(75)

        state = "middle"
        rospy.Subscriber('back', String, back_Callback)
        rospy.Subscriber('/drone/target', Twist, CommandCallback)
        drone_pub = rospy.Publisher('/drone/go', Empty, queue_size=1)
        

        rate = rospy.Rate(10)

        rospy.sleep(1)

        drone_pub.publish()

        go_flag = movebase_client(2,2,-2)
        
        # if the drone cannot see the landrobot and send a message, the landrobot will search for the bear
        while not rospy.is_shutdown():  
            if rospy.Time.now() > end_time and drone_saw == False:
                drone_saw = True
                # Start navigation
                movebase_client(1, 1, 0.5) 
            rate.sleep()  



    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
