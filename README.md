# The ASV System Package

The Autonomous Surface Vehicle (ASV) System Package is a collection of ROS packages I developed as a part of my master's thesis.


## Contents
This package contains:
+ `asv_ctrl_vo`: an implementation of the "Velocity Obstacle" algorithm for collision avoidance
+ `asv_msgs`
+ `asv_obstacle_tracker`: package that acts as a "black box", providing information about the states (and possibly metadata) that a collision avoidance system can subscribe to. _It does not actually track obstacles._
+ `asv_simulator`: simulates a nonlinear 3DOF surface vessel.
+ `asv_system`: metapackage with launch files
