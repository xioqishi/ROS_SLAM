#!/usr/bin/env python


#After cathing the bear, the ground robot will back to base

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
from std_msgs.msg import String, Empty , Int32
from rospy_tutorials.srv import *




#move to a point
def movebase_client(x,y):

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y  
    goal.target_pose.pose.orientation.w = 1
    goal.target_pose.pose.orientation.z = 0
    goal.target_pose.pose.orientation.x = 0   
    goal.target_pose.pose.orientation.y = 0  

    client.send_goal(goal)
    
    client.wait_for_result()
    #print(actionlib.SimpleGoalState.DONE)
    #return client.get_result()
    #print(client.wait_for_result())
    state = client.get_state()
    return state






def back_home(data):
    release_flag = 0
    #if the arm catch the bear, it returns 1 and the land robot back to home
    if data.data == 1:
        rospy.sleep(2)
        release_flag = movebase_client(0,0)
    #if the arm failed cathing the bear, it sends a message to catch again
    if data.data == 0:
        release_pub.publish()
        rospy.sleep(1)
        grab_more.publish()
        
    #When reaching home, it sends a message to relese the bear
    
    if release_flag == 3:
        tt = Empty()
        release_pub.publish(tt)



#initial node and topics
if __name__ == '__main__':
    try:
        rospy.init_node('back_home_node')
        rospy.Subscriber('/landrobot/torque',Int32,back_home)
        release_pub = rospy.Publisher('/landrobot/release', Empty, queue_size=2)
        grab_more = rospy.Publisher('/landrobot/grab', Empty, queue_size=2)
        rospy.spin()
 


    except rospy.ROSInterruptException:
        pass