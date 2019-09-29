#!/usr/bin/env python


#it publish the odom information between base_link and odom


import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

twisted_frame = Twist()

def CommandCallback(message_t):
    global twisted_frame
    twisted_frame = message_t

def pubPos():
    x = 0.0
    y = 0.0
    th = 0.0
    
    global twisted_frame
    current_time = rospy.Time.now()
    last_time = rospy.Time.now() 
    r = rospy.Rate(100)
    while not rospy.is_shutdown():
        vx = twisted_frame.linear.x
        vy = 0
        vth = twisted_frame.angular.z
                     
        current_time = rospy.Time.now()
        
        # compute odometry in a typical way given the velocities of the robot
        dt = (current_time - last_time).to_sec()
        delta_x = (vx * cos(th) - vy * sin(th)) * dt
        delta_y = (vx * sin(th) + vy * cos(th)) * dt
        delta_th = vth * dt

        x += delta_x
        y += delta_y
        th += delta_th
        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

        # first, we'll publish the transform over tf
        odom_broadcaster.sendTransform(
            (x, y, 0.),
            odom_quat,
            current_time,
            "base_link",
            "odom"
        )

        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

        # set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

        # publish the message
        odom_pub.publish(odom)

        last_time = current_time
        r.sleep()




if __name__ == "__main__":
    rospy.init_node('odometry_publisher')
    odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
    odom_broadcaster = tf.TransformBroadcaster()
    dennis_sub = rospy.Subscriber('/twist_mux/cmd_vel', Twist, CommandCallback)
    pubPos()
    rospy.spin()
