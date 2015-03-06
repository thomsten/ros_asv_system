#include "ros/ros.h"
#include <ros/console.h>

#include "asv_msgs/State.h"
#include "nav_msgs/OccupancyGrid.h"
#include <tf/transform_datatypes.h>

#include <iostream>
#include <math.h>

#include "asv_ctrl_vo/asv_ctrl_vo.h"


void rot2d(const Eigen::Vector2d &v, double yaw, Eigen::Vector2d &result);

VelocityObstacle::VelocityObstacle() : EDGE_SAMPLES_(10),
                                       VEL_SAMPLES_(41),
                                       ANG_SAMPLES_(101),
                                       GRID_RES_(0.1),
                                       RADIUS_(10.0),
                                       MAX_VEL_(4.0),
                                       MAX_ANG_(2.0944)
{
  VG_XLIM_ = 0;
  VG_YLIM_ = 0;

  asv_pose_ = Eigen::Vector3d(0.0, 0.0, 0.0);
  asv_twist_ = Eigen::Vector3d(0.0, 0.0, 0.0);

}

VelocityObstacle::~VelocityObstacle()
{
}

void VelocityObstacle::initialize(std::vector<asv_msgs::State> *obstacles)
{

  ROS_INFO("Initializing VO-node...");

  VG_XLIM_ = (int) 2*(MAX_VEL_/GRID_RES_ + 1);
  VG_YLIM_ = VG_XLIM_;//(int) 2*((MAX_VEL_/GRID_RES_)*sin(MAX_ANG_) + 1);

  ROS_INFO("VG_XLIM: %d, VG_YLIM: %d", VG_XLIM_, VG_YLIM_);

  vg_.header.frame_id = std::string("asv");
  vg_.info.map_load_time = ros::Time::now();
  vg_.info.resolution = GRID_RES_*10;
  vg_.info.width = VG_XLIM_;
  vg_.info.height = VG_YLIM_;
  // Set origin of window to be "in the middle"
  vg_.info.origin.position.x = - 0.5 * VG_XLIM_ * GRID_RES_*10;
  vg_.info.origin.position.y = - 0.5 * VG_YLIM_ * GRID_RES_*10;
  vg_.info.origin.orientation.w = 1.0;

  vg_.data.resize(VG_XLIM_*VG_YLIM_);

  obstacles_ = obstacles;

  ROS_INFO("Initialization complete!");
}

void VelocityObstacle::update(const ros::Publisher *og_pub)
{
  clearVelocityGrid();
  updateVelocityGrid();
  og_pub->publish(vg_);
}

void VelocityObstacle::updateVelocityGrid()
{
  double u0 = 0, u = 0;
  double theta0 = -MAX_ANG_, t = 0;
  double du = MAX_VEL_/VEL_SAMPLES_;
  double dtheta = 2*MAX_ANG_/ANG_SAMPLES_;

  for (std::vector<asv_msgs::State>::iterator it = obstacles_->begin();
       it != obstacles_->end();
       ++it) {

    Eigen::Vector3d obstacle_pose = Eigen::Vector3d(it->x, it->y, it->psi);
    Eigen::Vector3d obstacle_twist = Eigen::Vector3d(it->u, it->v, it->r);

    // Vector from obstacle position to asv position
    Eigen::Vector2d pab = -asv_pose_.head(2) + obstacle_pose.head(2);
    double alpha = asin(2*RADIUS_ / pab.norm());

    // Left and right bounds pointing inwards to the VO. (Guy et. al. 2009)
    Eigen::Vector2d lb, rb;
    rot2d(pab, -alpha + M_PI/2.0, lb);
    rot2d(pab, alpha - M_PI/2.0, rb);

    for (int u_it=0; u_it<VEL_SAMPLES_; ++u_it){
      for (int t_it=0; t_it<ANG_SAMPLES_; ++t_it){
        /// @todo
        u = u0 + u_it*du;
        t = theta0 + t_it*dtheta;

        if (inVelocityObstacle(u, t, lb, rb, obstacle_twist))
          {
            setVelocity(u, t, 100);
          }
        else if (violatesColregs(u, t, obstacle_pose, obstacle_twist))
          {
            setVelocity(u, t, 50);
          }
        else
          {
            setVelocity(u, t, 10);
          }
      }
    }
  }
}

bool VelocityObstacle::inVelocityObstacle(const double &u,
                                          const double &theta,
                                          const Eigen::Vector2d &lb,
                                          const Eigen::Vector2d &rb,
                                          const Eigen::Vector3d &obstacle_twist)
{
  Eigen::Vector2d vb = obstacle_twist.head(2);
  Eigen::Vector2d va(u*cos(theta), u*sin(theta));

  return ((va-vb).dot(lb) >= 0 && (va-vb).dot(rb) >= 0);
}

bool VelocityObstacle::violatesColregs(const double &u,
                                       const double &theta,
                                       const Eigen::Vector3d &obstacle_pose,
                                       const Eigen::Vector3d &obstacle_twist)
{
  Eigen::Vector2d pdiff = asv_pose_.head(2) - obstacle_pose.head(2);
  Eigen::Vector2d vdiff = Eigen::Vector2d(u*cos(theta), u*sin(theta)) - obstacle_twist.head(2);

  // Relative bearing (Loe, 2008)
  double alpha = atan2(pdiff[1], pdiff[0]) - obstacle_pose[2];
  double DEG2RAD = M_PI/180.0f;

  if (0.0 <= alpha or alpha < 15*DEG2RAD)
    {
      // Head-on: COLREGs applicaple if the following relation holds (Kuwata et. al., 2014)
      ROS_INFO_ONCE("Violates Head-On.");
      return (pdiff[0]*vdiff[1] - pdiff[1]*vdiff[0] < 0);
    }
  else if (15.0*DEG2RAD <= alpha or alpha < 112.5*DEG2RAD)
    {
      // Crossing from right
      ROS_INFO_ONCE("Violates Crossing from Right.");
      return (pdiff[0]*vdiff[1] - pdiff[1]*vdiff[0] < 0);
    }
  else if (247.5*DEG2RAD <= alpha or alpha < 345.0*DEG2RAD)
    {
      // Crossing from left: No COLREGs
      ROS_INFO("Crossing from Left.");
      return false;
    }
  else
    {
      // The remaining: Overtaking
      // 112.5*DEG2RAD <= alpha or alpha < 247.5*DEG2RAD
      ROS_INFO_ONCE("Violates Overtaking.");
      return (pdiff[0]*vdiff[1] - pdiff[1]*vdiff[0] < 0);
    }
}

void VelocityObstacle::clearVelocityGrid()
{
  for (int i=0; i<VG_XLIM_*VG_YLIM_; ++i)
    vg_.data[i] = 0;
}

void VelocityObstacle::setVelocity(const double &u, const double &theta, int val)
{
  setPoint(u*cos(theta), u*sin(theta), val);
}

/**
 * Set the value (val) of point (px,py) given relative to the body frame.
 */
void VelocityObstacle::setPoint(const double &px, const double &py, int val)
{
  int PX = (int) (px / GRID_RES_ + 0.5 * VG_XLIM_);
  int PY = (int) (py / GRID_RES_ + 0.5 * VG_YLIM_);

  if (PX >= VG_XLIM_ || PX < 0 || PY >= VG_YLIM_ || PY < 0 )
    {
      ROS_WARN("Attempted to set point outside VO-grid. (%.1f, %.1f), (%d, %d)", px, py, PX, PY);
      return;
    }
  else
    {
      if (val > 100)
        val = 100;
      else if (val < 0)
        val = 0;

      /// @TODO Unsure about this shit yo.
      int i = PY * VG_XLIM_ + PX;
      vg_.data[i] = val;
    }

}

void VelocityObstacle::updateAsvState(const nav_msgs::Odometry::ConstPtr &msg)
{
  double yaw = tf::getYaw(msg->pose.pose.orientation);
  asv_pose_[0] = msg->pose.pose.position.x;
  asv_pose_[1] = msg->pose.pose.position.y;
  asv_pose_[2] = yaw;
  asv_twist_[0] = msg->twist.twist.linear.x;
  asv_twist_[1] = msg->twist.twist.linear.y;
  asv_twist_[2] = msg->twist.twist.angular.z;
}


/////// UTILS /////////
void rot2d(const Eigen::Vector2d &v, double yaw, Eigen::Vector2d &result)
{
  Eigen::Matrix2d R;
  R << cos(yaw), -sin(yaw),
    sin(yaw), cos(yaw);
  result = R*v;
}

