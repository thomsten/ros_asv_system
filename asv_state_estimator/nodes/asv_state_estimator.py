#!/usr/bin/env python
import rospy

import tf
import geometry_msgs
import sensor_msgs

from tf.transformations import quaternion_from_euler as euler2quat
from nav_msgs.msg import Odometry


import numpy as np


class StateEstimator(object):
    def __init__(self, lat0, lon0, gps_topic="/gps", imu_topic="/imu"):
        self.ecef0 = np.zeros((3,1))

        self.lat0 = lat0
        self.lon0 = lon0
        self.cos_lon0 = np.cos(lon)
        self.cos_lat0 = np.cos(lat)
        self.sin_lon0 = np.sin(lon)
        self.sin_lat0 = np.sin(lat)
        self.ecef0 = self.geod2ecef(self.lat0, self.lon0, 0.0)

        self.odom = Odometry()
        self.first_gps_message = True

        self.gps_pos = np.zeros((2,1))
        self.imu_ang = geometry_msgs.Quaternion()
        self.imu_angvel = geometry_msgs.Vector3()

        self.gps_sub = rospy.Subscriber(gps_topic, sensor_msgs.NavSatFix, self.gpsCallback)
        self.imu_sub = rospy.Subscriber(imu_topic, sensor_msgs.Imu, self.imuCallback)
        self.odom_pub = rospy.Publisher("/asv/state", Odometry, queue_size=10)

    def geod2ecef(self, lat, lon, h):
        sin_lat = np.sin(lat)
        sin_lon = np.sin(lon)
        cos_lat = np.cos(lat)
        cos_lon = np.cos(lon)

        a = 6378137
        f = 1/298.257223563
        b = a*(1 - f)
        e2 = 1 - (b/a)*(b/a);

        Nphi = a / np.sqrt(1 - e2*np.power(sin_lat, 2))

        if h == np.nan:
            h = 0

        ecef = np.array([(Nphi + h)*cos_lat*cos_lon,
                         (Nphi + h)*cos_lat*sin_lon,
                         (Nphi*(1 - e2) + h)*sin_lat])
        return ecef

    def ecef2ned(self, ecef):
        p_ecef = ecef - self.ecef0

        phiP = np.arctan2(ecef[2], np.sqrt(ecef[0]**2 + ecef[1]**2))

        sin_phiP = np.sin(phiP)
        cos_phiP = np.cos(phiP)

        rot = np.array([[-sin_phiP*self.cos_lon0, -sin_phiP*self.sin_lon0, cos_phiP],
                        [-self.sin_lon0, self.cos_lon0, .0],
                        [cos_phiP*self.cos_lon0, cos_phiP*self.sin_lon0, sin_phiP]])


        p_ned = np.dot(rot, p_ecef)
        return p_ned


    def gpsCallback(self, msg):
        lat = msg.latitude
        lon = msg.longitude
        h = msg.height

        ecef = self.geod2ecef(lat, lon, h)

        p_ned = self.ecef2ned(ecef)

        self.gps_pos = p_ned[0:2]

    def imuCallback(self, msg):
        self.imu_ang = msg.orientation
        self.imu_angvel = msg.angular_velocity

    def update_orientation(self):
        ## @todo Filter yaw
        self.odom.pose.pose.orientation = self.imu_ang

    def update(self):
        self.odom.pose.pose.position.x = self.gps_pos[0]
        self.odom.pose.pose.position.y = self.gps_pos[1]

        self.update_orientation()

        self.odom.twist.twist.angular = self.imu_angvel

        self.odom_pub.publish(self.odom)

if __name__ == "__main__":

    rospy.init_node("state_estimator")

    # The dock outside DNV GL Høvik
    lat0 = 59.88765
    lon0 = 10.56594

    state_estimator = StateEstimator(lat0, lon0, gps_topic="/gps", imu_topic="/imu")

    r = rospy.Rate(0.1)
    while not rospy.is_shutdown():

        state_estimator.update()

        r.sleep()
