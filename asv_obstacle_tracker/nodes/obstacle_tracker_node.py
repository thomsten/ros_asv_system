#!/usr/bin/env python
"""An 'obstacle tracker' module.

In reality this module just subscribes to all obstacle ships' odometry data and
combine them into a single list of states for the ASV to subscribe to. This
design allows for, e.g., adding noise to the 'measurements'.

"""


import rospy

from nav_msgs.msg import Odometry
from asv_msgs.msg import State, StateArray

import numpy as np

import matplotlib.pyplot as plt

class WaveFilter(object):
    """Second order filter for simulating wave (or other) noise.

    The filter is on the form:
               y(s)       2*lambda0*omega0*sigma*s
        h(s) = ---- = -------------------------------------,
               w(s)    s^2 + 2*lambda0*omega0*s + omega0^2
    where w(s) is Gaussian white noise.
    """
    def __init__(self, wf_gain=1.0, sigma=0.2344, omega0=0.7823, lambda0=0.0862):
        self.A = np.array([[0, 1], [ -omega0**2, -2*lambda0*omega0]])
        self.B = np.array([[0],[2*lambda0*omega0*sigma]])
        self.C = np.array([[0, wf_gain]])
        self.state = np.zeros((2,1))

        self.dT = 0.05

    def getWaveNoise(self):
        wn = (np.random.random_sample(1)) - 0.5
        self.state += (np.dot(self.A, self.state) + self.B*wn)*self.dT

        return np.dot(self.C, self.state)

def quat2yaw(q):
    return np.arctan2(2*(q[0]*q[1] + q[2]*q[3]), 1 - 2*(q[1]**2 + q[2]**2))


def obstacleCallback(data, num):
    global wf
    q = (data.pose.pose.orientation.x,
         data.pose.pose.orientation.y,
         data.pose.pose.orientation.z,
         data.pose.pose.orientation.w)

    statearray.states[num].x = data.pose.pose.position.x
    statearray.states[num].y = data.pose.pose.position.y
    statearray.states[num].psi = quat2yaw(q) + wf[num].getWaveNoise()
    statearray.states[num].u = data.twist.twist.linear.x
    statearray.states[num].v = data.twist.twist.linear.y
    statearray.states[num].r = data.twist.twist.angular.z

    pub.publish(statearray)

if __name__ == "__main__":
    global wf
    wf = []

    rospy.init_node("obstacle_tracker_node")

    ships = rospy.get_param("/obstacles")

    wf_gain = rospy.get_param("wave_filter_gain", 0.0)
    rospy.loginfo("Using wave gain: %f", wf_gain)

    statearray = StateArray()

    subscriber_list = []
    num = 0

    for ship in ships:
        statearray.states.append(State())
        subscriber_list.append(rospy.Subscriber("/obstacles/" + str(ship) + "/state", Odometry, obstacleCallback, num))
        statearray.states[num].header.id = num
        statearray.states[num].header.name = str(ship)
        statearray.states[num].header.radius = ships[str(ship)][str(ship)]['radius']
        wf.append(WaveFilter(wf_gain))
        num += 1

    pub = rospy.Publisher("/obstacle_states", StateArray, queue_size=1)

    rospy.spin()
