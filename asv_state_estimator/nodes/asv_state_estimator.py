#!/usr/bin/env python
import rospy
import numpy as np

import tf

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu

from tf.transformations import quaternion_from_euler as euler2quat


DEG2RAD = np.pi / 180.0

def quat2yaw(q):
    return np.arctan2(2*(q[0]*q[1] + q[2]*q[3]), 1 - 2*(q[1]**2 + q[2]**2))

def normalize_angle(angle):
    """Ensure angle within [-PI, PI)"""
    while angle > np.pi:
        angle -= 2.0*np.pi
    while angle <= -np.pi:
        angle += 2.0*np.pi
    return angle

class StateEstimator(object):
    def __init__(self, lat0, lon0, fix_topic="/fix", vel_topic="/vel", imu_topic="/imu"):
        self.ecef0 = np.zeros((3,1))

        self.lat0 = lat0
        self.lon0 = lon0
        self.cos_lon0 = np.cos(lon0)
        self.cos_lat0 = np.cos(lat0)
        self.sin_lon0 = np.sin(lon0)
        self.sin_lat0 = np.sin(lat0)
        self.ecef0 = self.geod2ecef(self.lat0, self.lon0, 0.0)

        self.odom = Odometry()
        self.odom.header.frame_id = "map"
        self.odom.child_frame_id = "asv"
        self.first_gps_message = True

        self.gps_pos = np.zeros((2,1))
        self.gps_vel = np.zeros((2,1))
        self.imu_ang = np.array([0,0,0,1])
        self.imu_angvel = np.zeros((3,1))
        self.yaw_correction = 0.0

        self.gps_fix = False
        self.gps_course = 0.0


        self.fix_sub = rospy.Subscriber(fix_topic, NavSatFix, self.fixCallback)
        self.vel_sub = rospy.Subscriber(vel_topic, TwistStamped, self.velCallback)
        self.imu_sub = rospy.Subscriber(imu_topic, Imu, self.imuCallback)

        self.odom_pub = rospy.Publisher("/asv/state", Odometry, queue_size=10)
        self.tf_br = tf.TransformBroadcaster()

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

    def ned2enu(self, ned):
        """Convert NED position to ENU position"""
        if len(ned) < 3:
            return np.array([ned[1], ned[0]])
        else:
            return np.array([ned[1], ned[0], -ned[2]])

    def geod2enu(self, lat, lon, h):
        ecef = self.geod2ecef(lat, lon, h)

        ned = self.ecef2ned(ecef)
        enu = self.ned2enu(ned)
        return enu

    def fixCallback(self, msg):
        lat = msg.latitude * DEG2RAD
        lon = msg.longitude * DEG2RAD
        h = msg.altitude

        if msg.status.status < 0:
            self.gps_fix = False
            return
        else:
            self.gps_fix = True

        enu = geod2enu(self, lat, lon, h)

        self.gps_pos = enu[0:2]

    def velCallback(self, msg):
        self.gps_vel[0] = msg.twist.linear.x
        self.gps_vel[1] = msg.twist.linear.y
        self.gps_course = np.arctan2(self.gps_vel[1], self.gps_vel[0])

    def imuCallback(self, msg):
        self.imu_ang = np.array([msg.orientation.x,
                                 msg.orientation.y,
                                 msg.orientation.z,
                                 msg.orientation.w])
        self.imu_angvel = np.array([msg.angular_velocity.x,
                                    msg.angular_velocity.y,
                                    msg.angular_velocity.z])

    def update_orientation(self):
        # Update yaw correction if gps fix is ok, speed is above 0.5 m/s and yaw rate below 0.5 m/s

        velocity = np.sqrt(self.odom.twist.twist.linear.x**2 + self.odom.twist.twist.linear.y**2)
        if (self.gps_fix and \
            velocity > 0.5 and \
            self.odom.twist.twist.angular.z < 0.5):
            self.yaw_correction = normalize_angle(self.gps_course - quat2yaw(self.imu_ang))

        q = self.imu_ang #+ euler2quat(0,0,self.yaw_correction)
        q = q / np.linalg.norm(q)

        self.odom.pose.pose.orientation = Quaternion(*q)

        return q

    def update(self):
        self.odom.pose.pose.position.x = self.gps_pos[0]
        self.odom.pose.pose.position.y = self.gps_pos[1]
        self.odom.twist.twist.linear.x = self.gps_vel[0]
        self.odom.twist.twist.linear.y = self.gps_vel[1]
        self.odom.twist.twist.angular = Vector3(*self.imu_angvel)

        q = self.update_orientation()


        self.odom.header.stamp = rospy.Time.now()

        self.tf_br.sendTransform((self.gps_pos[0], self.gps_pos[1], 0.0),
                                 q,
                                 rospy.Time.now(),
                                 "asv",
                                 "map")
        self.odom_pub.publish(self.odom)
        self.odom.header.seq += 1

if __name__ == "__main__":

    rospy.init_node("state_estimator")

    # The dock outside DNV GL Hovik
    lat0 = 59.88765 * DEG2RAD
    lon0 = 10.56594 * DEG2RAD

    state_estimator = StateEstimator(lat0, lon0, fix_topic="/fix", vel_topic="/vel", imu_topic="/imu")

    r = rospy.Rate(10)
    while not rospy.is_shutdown():

        state_estimator.update()

        r.sleep()
