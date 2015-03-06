#!/usr/bin/env python

import rospy, tf

from nav_msgs.msg import Odometry
from asv_msgs.msg import State, StateArray

import numpy as np



def obstacleCallback(data):
    q = (data.pose.pose.orientation.x,
         data.pose.pose.orientation.y,
         data.pose.pose.orientation.z,
         data.pose.pose.orientation.w)
    r,p,y = tf.transformations.euler_from_quaternion(q)

    statearray.states[0].x = data.pose.pose.position.x
    statearray.states[0].y = data.pose.pose.position.y
    statearray.states[0].psi = y
    statearray.states[0].u = data.twist.twist.linear.x
    statearray.states[0].v = data.twist.twist.linear.y
    statearray.states[0].r = data.twist.twist.angular.z

    pub.publish(statearray)

if __name__ == "__main__":
    rospy.init_node("obstacle_tracker_node")

    statearray = StateArray()
    statearray.states.append(State())

    pub = rospy.Publisher("/obstacle_states", StateArray, queue_size=10)
    sub = rospy.Subscriber("/obstacles/ship1/state", Odometry, obstacleCallback)


    rospy.spin()
