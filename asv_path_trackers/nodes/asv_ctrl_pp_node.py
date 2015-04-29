#!/usr/bin/env python
import numpy as np
import rospy
import geometry_msgs.msg
import nav_msgs.msg
from visualization_msgs.msg import Marker

from utils import Controller

class PurePursuitROS(object):
    """A ROS wrapper for PurePursuit()."""
    def __init__(self, rate, R2=20**2, u_d=3.0, switch_criterion='circle'):
        self.controller = PurePursuit(R2, u_d, switch_criterion)
        self.rate = rate
        self.wp   = self.controller.wp
        self.nwp  = 0
        self.cwp  = 0

        self._cmd_publisher   = rospy.Publisher("cmd_vel", geometry_msgs.msg.Twist, queue_size=1)
        self._odom_subscriber = rospy.Subscriber("state", nav_msgs.msg.Odometry, self._odom_callback, queue_size=1)
        self._wps_publisher   = rospy.Publisher("waypoints", Marker, queue_size=10)


        self.odom = nav_msgs.msg.Odometry()
        self.cmd  = geometry_msgs.msg.Twist()
        self.cmd.linear.x = u_d

        self._first_draw = True

    def set_waypoints(self, wps):
        self.wp = wps
        self.controller.wp = np.copy(wps)
        self.controller.nWP = len(wps)
        self.controller.wp_initialized = True
        self.nwp = len(wps)

    def _visualize_waypoints(self, switched):
        if not switched and not self._first_draw:
            return

        if self._first_draw:
            for wp in range(1, self.nwp):
                mk = Marker()
                mk.header.seq += 1
                mk.header.frame_id = "map"
                mk.header.stamp = rospy.Time.now()

                mk.ns = "waypoints"
                mk.id = wp
                mk.type = Marker.CYLINDER
                D = np.sqrt(self.controller.R2)
                mk.scale.x = D
                mk.scale.y = D
                mk.scale.z = 2. # height [m]
                mk.action = Marker.ADD

                mk.pose = geometry_msgs.msg.Pose()
                mk.pose.position.x = self.wp[wp,0]
                mk.pose.position.y = self.wp[wp,1]
                mk.pose.orientation.w = 1

                mk.lifetime = rospy.Duration()
                mk.color.a = .3
                mk.color.r = 0.
                mk.color.g = 0.
                mk.color.b = 0.

                if wp == self.cwp:
                    mk.color.g = 1.
                else:
                    mk.color.r = 1.

                self._wps_publisher.publish(mk)
        else:
            for wp in [self.cwp-1, self.cwp]:
                mk = Marker()
                mk.header.seq += 1
                mk.header.frame_id = "map"
                mk.header.stamp = rospy.Time.now()

                mk.ns = "waypoints"
                mk.id = wp
                mk.type = Marker.CYLINDER
                D = np.sqrt(self.controller.R2)*2
                mk.scale.x = D
                mk.scale.y = D
                mk.scale.z = 2. # height [m]
                mk.action = Marker.ADD

                mk.pose = geometry_msgs.msg.Pose()
                mk.pose.position.x = self.wp[wp,0]
                mk.pose.position.y = self.wp[wp,1]
                mk.pose.orientation.w = 1

                mk.lifetime = rospy.Duration()
                mk.color.a = .3
                mk.color.r = 0.
                mk.color.g = 0.
                mk.color.b = 0.

                if wp == self.cwp:
                    mk.color.g = 1.
                else:
                    mk.color.r = 1.



                self._wps_publisher.publish(mk)

        self._first_draw = True

    def _odom_callback(self, data):
        self.odom = data

    def _update(self):

        u_d, psi_d, switched = self.controller.update(self.odom.pose.pose.position.x,
                                                      self.odom.pose.pose.position.y)
        if switched:
            print "Switched!"
            self.cwp += 1

        # Publish cmd_vel
        self.cmd.linear.x = u_d
        self.cmd.angular.y = psi_d
        self.cmd.angular.z = 0.0

        self._cmd_publisher.publish(self.cmd)

        self._visualize_waypoints(switched)

    def run_controller(self):
        r = rospy.Rate(1.0/self.rate)

        while not rospy.is_shutdown():
            self._update()
            r.sleep()

class PurePursuit(Controller):
    """This class implements the classic LOS guidance scheme."""
    def __init__(self, R2=20**2, u_d = 3.0, switch_criterion='circle'):
        self.R2 = R2 # Radii of acceptance (squared)
        self.R  = np.sqrt(R2)

        self.cWP = 0 # Current waypoint
        self.wp = None
        self.nWP = 0
        self.wp_initialized = False

        if switch_criterion == 'circle':
            self.switching_criterion = self.circle_of_acceptance
        elif switch_criterion == 'progress':
            self.switching_criterion = self.progress_along_path

        self.u_d = u_d

    def __str__(self):
        return """Radii: %f\nCurrent Waypoint: %d"""%(self.R, self.cWP)

    def circle_of_acceptance(self, x, y):
        return \
            (x - self.wp[self.cWP][0])**2 + \
            (y - self.wp[self.cWP][1])**2 < self.R2

    def progress_along_path(self, x, y):
        return \
            np.abs((self.wp[self.cWP][0] - x)*np.cos(self.Xp) + \
                   (self.wp[self.cWP][1] - y)*np.sin(self.Xp)) < self.R

    def update(self, x, y):
        if not self.wp_initialized:
            print "Error. No waypoints!"
            return 0,0,False

        xk = self.wp[self.cWP][0]
        yk = self.wp[self.cWP][1]

        psi_d = np.arctan2(yk-y , xk-x)


        if self.R2 > 999999:
            return 0, psi_d, False

        switched = False

        if self.switching_criterion(x, y):
            while self.switching_criterion(x,y):
                if self.cWP < self.nWP - 1:
                    self.cWP += 1
                    switched = True
                else:
                    # Last waypoint reached
                    if self.R2 < 999999:
                        print "Waypoint %d: (%.2f, %.2f) reached!" % (self.cWP,
                                                                      self.wp[self.cWP][0],
                                                                      self.wp[self.cWP][1])
                        print "Last Waypoint reached!"
                        self.R2 = np.Inf
                        return 0, psi_d, False



        return self.u_d, psi_d, switched

if __name__ == "__main__":
    rospy.init_node("PurePursuit_guidance_controller")

    waypoints = rospy.get_param("~waypoints")
    u_d = rospy.get_param("~u_d")

    guide = PurePursuitROS(.2, u_d=u_d)

    wps = np.array(waypoints)
    guide.set_waypoints(wps)

    guide.run_controller()
