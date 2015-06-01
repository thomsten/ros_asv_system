# The ASV System Package

The Autonomous Surface Vehicle (ASV) System Package is a collection of ROS
packages I developed as a part of my master's thesis.


## Contents
This package contains:
+ `asv_ctrl_vo`: an implementation of the "Velocity Obstacle" algorithm for
collision avoidance.
+ `asv_path_trackers`: implements the (Integral) Line of Sight (LOS) method and
a simple pure pursuit scheme for path following.
+ `asv_msgs`: message types used in the system.
+ `asv_obstacle_tracker`: package that acts as a "black box", providing
information about the states (and possibly metadata) that a collision avoidance
system can subscribe to. _It does not actually track obstacles._ It is also
possible to simulate the addition of sensor noise using this package.
+ `asv_simulator`: simulates a nonlinear 3DOF surface vessel.
+ `asv_system`: metapackage with launch files and more!
+ `state_estimator`: unfinished package for estimating the ASV pose given GPS
and IMU data.
