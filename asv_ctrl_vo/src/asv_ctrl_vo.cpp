#include "ros/ros.h"
#include <ros/console.h>

#include "asv_msgs/State.h"
#include "nav_msgs/OccupancyGrid.h"
#include "visualization_msgs/Marker.h"
#include <tf/transform_datatypes.h>

#include <iostream>
#include <math.h>

#include "asv_ctrl_vo/asv_ctrl_vo.h"


void rot2d(const Eigen::Vector2d &v, double yaw, Eigen::Vector2d &result);

VelocityObstacle::VelocityObstacle() : EDGE_SAMPLES_(10),
                                       VEL_SAMPLES_(41),
                                       ANG_SAMPLES_(101),
                                       GRID_RES_(0.1),
                                       RADIUS_(5.0),
                                       MAX_VEL_(4.0),
                                       MAX_ANG_(2.0944),
                                       MIN_DIST_(100.0)
{
  asv_pose_ = Eigen::Vector3d(0.0, 0.0, 0.0);
  asv_twist_ = Eigen::Vector3d(0.0, 0.0, 0.0);

}

VelocityObstacle::~VelocityObstacle()
{
}

void VelocityObstacle::initialize(std::vector<asv_msgs::State> *obstacles)
{

  ROS_INFO("Initializing VO-node...");

  vo_grid_.resize(ANG_SAMPLES_*VEL_SAMPLES_);

  obstacles_ = obstacles;

  ROS_INFO("Initialization complete!");
}

void VelocityObstacle::update()
{
  clearVelocityGrid();
  updateVelocityGrid();
}

void VelocityObstacle::updateVelocityGrid()
{
  const double RAD2DEG = 180.0/M_PI;

  double u0 = 0, u = 0;
  double theta0 = -MAX_ANG_ + asv_pose_[2], t = 0;
  double du = MAX_VEL_/VEL_SAMPLES_;
  double dtheta = 2*MAX_ANG_/ANG_SAMPLES_;

  Eigen::Vector2d va_ref = Eigen::Vector2d(u_d_,
                                           psi_d_);
  Eigen::Vector2d v_new_ref = Eigen::Vector2d(0.0,0.0);

  std::vector<asv_msgs::State>::iterator it;

  for (it = obstacles_->begin(); it != obstacles_->end(); ++it) {
    Eigen::Vector3d obstacle_pose = Eigen::Vector3d(it->x, it->y, it->psi);
    Eigen::Vector3d obstacle_twist = Eigen::Vector3d(it->u, it->v, it->r);

    // Vector from obstacle position to asv position
    Eigen::Vector2d pab = -asv_pose_.head(2) + obstacle_pose.head(2);

    if (pab.norm() < MIN_DIST)
      continue;

    double alpha = asin(2*RADIUS_ / pab.norm());

    // Left and right bounds pointing inwards to the VO. (Guy et. al. 2009)
    Eigen::Vector2d lb, rb;
    pab = pab/pab.norm();
    rot2d(pab,  alpha - 0.5*M_PI, lb);
    rot2d(pab, -alpha + 0.5*M_PI, rb);

    Eigen::Vector2d vb;
    rot2d(obstacle_twist.head(2), obstacle_pose[2], vb);

    // Objective function value. By minimizing this, an "optimal" velocity may be selected.
    double objval = 0.0;


    for (int u_it=0; u_it<VEL_SAMPLES_; ++u_it){
      for (int t_it=0; t_it<ANG_SAMPLES_; ++t_it){
        /// @todo
        u = u0 + u_it*du;
        t = theta0 + t_it*dtheta;

        v_new_ref[0] = u;
        v_new_ref[1] = t;

        objval = (v_new_ref - va_ref).norm() / 16.0;

        if (inVelocityObstacle(u, t, lb, rb, vb))
          {
            setVelocity(u_it, t_it, 1.0);
          }
        else if (violatesColregs(u, t, obstacle_pose, vb))
          {
            setVelocity(u_it, t_it, 0.75);
          }
        else
          {
            // @todo This line will cause a bug if multiple obstacles are present.
            setVelocity(u_it, t_it, objval);
          }
      }
    }
  }
}

bool VelocityObstacle::inVelocityObstacle(const double &u,
                                          const double &theta,
                                          const Eigen::Vector2d &lb,
                                          const Eigen::Vector2d &rb,
                                          const Eigen::Vector2d &vb)
{
  Eigen::Vector2d va(u*cos(theta), u*sin(theta));

  return ((va-vb).dot(lb) >= 0 && (va-vb).dot(rb) >= 0);
}

bool VelocityObstacle::violatesColregs(const double &u,
                                       const double &theta,
                                       const Eigen::Vector3d &obstacle_pose,
                                       const Eigen::Vector2d &vb)
{
  Eigen::Vector2d pdiff = asv_pose_.head(2) - obstacle_pose.head(2);
  Eigen::Vector2d vdiff = Eigen::Vector2d(u*cos(theta), u*sin(theta)) - vb;

  // Relative bearing (Loe, 2008)
  double alpha = atan2(pdiff[1], pdiff[0]) - obstacle_pose[2];
  const double DEG2RAD = M_PI/180.0f;

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
  for (int i=0; i<vo_grid_.size(); ++i)
    vo_grid_[i] = 0.0;
}

void VelocityObstacle::setVelocity(const int &ui, const int &ti, const double &val)
{
  if (marker_ != NULL)
    {
      // Convert from scalar value to RGB heatmap: https://www.particleincell.com/2014/colormap/
      double newval = 4*(1.0 - val);

      // Get integer part
      double group = floor(newval);
      double frac = (newval - group);

      double r=0.0, g=0.0, b=0.0;

      switch ((int) group)
        {
        case 0:
          r = 1.0; g = frac; b = 0;
          break;
        case 1:
          r = 1.0-frac; g = 1.0; b = 0;
          break;
        case 2:
          r = 0; g = 1.0; b = frac;
          break;
        case 3:
          r = 0; g = 1.0-frac; b = 1.0;
          break;
        case 4:
          r = 0; g = 0; b = 1.0;
          break;
        }

      marker_->colors[ui*ANG_SAMPLES_ + ti].r = r;
      marker_->colors[ui*ANG_SAMPLES_ + ti].g = g;
      marker_->colors[ui*ANG_SAMPLES_ + ti].b = b;
    }

  vo_grid_[ui*ANG_SAMPLES_ + ti] = val;
}

void VelocityObstacle::updateAsvState(const nav_msgs::Odometry::ConstPtr &msg, const double &u_d, const double &psi_d)
{
  double yaw = tf::getYaw(msg->pose.pose.orientation);
  asv_pose_[0] = msg->pose.pose.position.x;
  asv_pose_[1] = msg->pose.pose.position.y;
  asv_pose_[2] = yaw;
  asv_twist_[0] = msg->twist.twist.linear.x;
  asv_twist_[1] = msg->twist.twist.linear.y;
  asv_twist_[2] = msg->twist.twist.angular.z;

  u_d_ = u_d;
  psi_d_ = psi_d;
}

void VelocityObstacle::initializeMarker(visualization_msgs::Marker *marker)
{
  marker_ = marker;
  double u0 = 0, u = 0;
  double theta0 = -MAX_ANG_, t = 0;
  double du = MAX_VEL_/VEL_SAMPLES_;
  double dtheta = 2*MAX_ANG_/ANG_SAMPLES_;

  const double MFACTOR = 2.0;

  for (int ui=0; ui < VEL_SAMPLES_; ++ui) {
    for (int ti=0; ti < ANG_SAMPLES_; ++ti){
      u = u0 + ui*du;
      t = theta0 + ti*dtheta;
      marker_->points[ui*ANG_SAMPLES_ + ti].x = MFACTOR*u*cos(t);
      marker_->points[ui*ANG_SAMPLES_ + ti].y = MFACTOR*u*sin(t);
      marker_->colors[ui*ANG_SAMPLES_ + ti].a = 1.0;
    }
  }
}

void VelocityObstacle::getBestControlInput(double &u_best, double &psi_best)
{

  int min = -1;
  double minval = 9999.9;
  // Find (position of) minima
  for (int i=0; i < ANG_SAMPLES_*VEL_SAMPLES_; ++i) {
    if (vo_grid_[i] < minval) {
      minval = vo_grid_[i];
      min = i;
    }
  }

  int ui = min / ANG_SAMPLES_;
  int ti = min % ANG_SAMPLES_;

  double du = MAX_VEL_/VEL_SAMPLES_;
  double dtheta = 2*MAX_ANG_/ANG_SAMPLES_;


  u_best = ui*du;
  psi_best = -MAX_ANG_ + ti*dtheta + asv_pose_[2];

  // ROS_INFO("Best control input: (%.1f, %.1f), (%.1f, %.1f), (%d, %d), %d",
  //          u_best,
  //          psi_best,
  //          u_d_,
  //          psi_d_,
  //          ui,
  //          ti,
  //          min);

}

/////// UTILS /////////
void rot2d(const Eigen::Vector2d &v, double yaw, Eigen::Vector2d &result)
{
  Eigen::Matrix2d R;
  R << cos(yaw), -sin(yaw),
    sin(yaw), cos(yaw);
  result = R*v;
}

