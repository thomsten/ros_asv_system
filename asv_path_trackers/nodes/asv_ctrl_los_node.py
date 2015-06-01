#!/usr/bin/env python
"""(Integral) Line of Sight implementation for ROS

This implementation is based on [1].

TODO's:
- The LOS-controller should use the initial position as the first 'waypoint'

[1]: Handbook of Marine Craft Hydrodynamics and Motion Control. T.I. Fossen, 2011.

"""

import numpy as np
import rospy
import geometry_msgs.msg
import nav_msgs.msg
from visualization_msgs.msg import Marker
from utils import Controller

class LOSGuidanceROS(object):
    """A ROS wrapper for LOSGuidance()"""
    def __init__(self,
                 R2=20**2,
                 u_d=2.0,
                 de=50.0,
                 Ki=0.0,
                 dt=0.2,
                 max_integral_correction=np.pi*20.0/180.0,
                 switch_criterion='circle'):

        self.controller = LOSGuidance(R2,
                                      u_d,
                                      de,
                                      Ki,
                                      dt,
                                      max_integral_correction,
                                      switch_criterion)
        self.rate = dt
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
                D = np.sqrt(self.controller.R2) * 2
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
        r = rospy.Rate(1/self.rate)

        while not rospy.is_shutdown():
            self._update()
            r.sleep()

class LOSGuidance(Controller):
    """This class implements the classic LOS guidance scheme."""
    def __init__(self,
                 R2=20**2,
                 u_d=2.0,
                 de=50.0,
                 Ki=0.0,
                 dt=0.2,
                 max_integral_correction=np.pi*20.0/180.0,
                 switch_criterion='circle'):
        self.R2 = R2 # Radii of acceptance (squared)
        self.R  = np.sqrt(R2)
        self.de = de # Lookahead distance

        self.dt = dt
        self.max_integral_correction = np.abs(np.tan(max_integral_correction) * de)
        self.Ki = Ki

        self.e_integral = 0.0

        self.cWP = 0 # Current waypoint
        self.wp = None
        self.nWP = 0
        self.wp_initialized = False

        if switch_criterion == 'circle':
            self.switching_criterion = self.circle_of_acceptance
        elif switch_criterion == 'progress':
            self.switching_criterion = self.progress_along_path

        self.Xp = 0.0
        self.u_d = u_d

    def __str__(self):
        return """Radii: %f\nLookahead distance: %f\nCurrent Waypoint: %d"""%(self.R, self.de, self.cWP)

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

        if self.R2 > 999999:
            # Last waypoint has been reached.
            return 0, self.Xp, False

        #print self.wp[self.cWP,:], str(self)
        switched = False

        if self.switching_criterion(x, y):
            while self.switching_criterion(x,y):
                if self.cWP < self.nWP - 1:
                # There are still waypoints left
                    print "Waypoint %d: (%.2f, %.2f) reached!" % (self.cWP,
                                                                  self.wp[self.cWP][0],
                                                                  self.wp[self.cWP][1])
                    new_course = np.arctan2(self.wp[self.cWP + 1][1] - self.wp[self.cWP][1],
                                            self.wp[self.cWP + 1][0] - self.wp[self.cWP][0])

                    if (np.abs(new_course - self.Xp) > np.pi/4.0):
                        self.e_integral = 0.0

                    self.Xp = new_course
                    self.cWP += 1
                    switched = True
                else:
                    # Last waypoint reached

                    if self.R2 < 50000:
                        print "Waypoint %d: (%.2f, %.2f) reached!" % (self.cWP,
                                                                      self.wp[self.cWP][0],
                                                                      self.wp[self.cWP][1])
                        print "Last Waypoint reached!"
                        self.R2 = np.Inf
                    return 0, self.Xp, False

        xk = self.wp[self.cWP][0]
        yk = self.wp[self.cWP][1]

        # Cross-track error Eq. (10.10), [Fossen, 2011]
        e  = -(x - xk)*np.sin(self.Xp) + (y - yk)*np.cos(self.Xp)
        self.e_integral += e*self.dt

        if self.e_integral*self.Ki > self.max_integral_correction:
            self.e_integral -= e*self.dt

        Xr = np.arctan2( -(e + self.Ki*self.e_integral), self.de)

        psi_d = self.Xp + Xr

        return self.u_d, psi_d, switched

if __name__ == "__main__":
    rospy.init_node("LOS_Guidance_controller")

    waypoints = rospy.get_param("~waypoints")
    u_d = rospy.get_param("~u_d", 2.0)
    R2 = rospy.get_param("~acceptance_radius", 20)**2
    dt = rospy.get_param("~update_rate", .2)
    de = rospy.get_param("~lookahead_distance", 40.0)
    Ki = rospy.get_param("~integral_gain", 0.0)
    max_integral_correction = rospy.get_param("~max_integral_correction", np.pi*20/180)

    guide = LOSGuidanceROS(R2,
                           u_d,
                           de,
                           Ki,
                           dt,
                           max_integral_correction,
                           switch_criterion='circle')

    wps = np.array(waypoints)
    guide.set_waypoints(wps)

    guide.run_controller()
