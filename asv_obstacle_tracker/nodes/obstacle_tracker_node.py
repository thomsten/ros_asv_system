#!/usr/bin/env python

import rospy

from nav_msgs.msg import Odometry
from asv_msgs.msg import State, StateArray

import numpy as np

import matplotlib.pyplot as plt

class WaveFilter(object):
    def __init__(self):
        sigma   = 0.2344
        omega0  = 0.7823
        lambda0 = 0.0862
        gain    = 0.5
        self.A = np.array([[0, 1], [ -omega0**2, -2*lambda0*omega0]])
        self.B = np.array([[0],[1]])
        self.C = np.array([[0, gain]])
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
    statearray.states[num].psi = quat2yaw(q) #+ wf.getWaveNoise()
    statearray.states[num].u = data.twist.twist.linear.x
    statearray.states[num].v = data.twist.twist.linear.y
    statearray.states[num].r = data.twist.twist.angular.z

    pub.publish(statearray)

if __name__ == "__main__":
    # fig = plt.figure()
    # ax = fig.add_subplot(111)
    # wf = WaveFilter()
    # samples = np.zeros(10000)
    # for i in range(0,len(samples)):
    #     samples[i] = wf.getWaveNoise()
    # ax.plot(samples*180/np.pi)

    # plt.show()
    global wf
    wf = WaveFilter()

    rospy.init_node("obstacle_tracker_node")

    ships = rospy.get_param("/obstacles")


    statearray = StateArray()

    subscriber_list = []
    num = 0

    for ship in ships:
        statearray.states.append(State())
        subscriber_list.append(rospy.Subscriber("/obstacles/" + str(ship) + "/state", Odometry, obstacleCallback, num))
        statearray.states[num].header.id = num
        statearray.states[num].header.name = str(ship)
        statearray.states[num].header.radius = ships[str(ship)][str(ship)]['radius']
        num += 1

    pub = rospy.Publisher("/obstacle_states", StateArray, queue_size=1)

    rospy.spin()
