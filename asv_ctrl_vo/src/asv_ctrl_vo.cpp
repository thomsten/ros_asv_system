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
  double theta0 = -MAX_ANG_ + asv_pose_[2], t = 0;
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

    pab = pab/pab.norm();

    rot2d(pab,  alpha - 0.5*M_PI, lb);
    rot2d(pab, -alpha + 0.5*M_PI, rb);

    const double RAD2DEG = 180.0/M_PI;

    Eigen::Vector2d vb;
    rot2d(obstacle_twist.head(2), obstacle_pose[2], vb);

    // ROS_INFO("lb: (%.3f, %.3f), rb: (%.3f, %.3f), vb: (%.2f, %.2f), nub: (%.2f, %.2f)",
    //          lb[0], lb[1], rb[0], rb[1], vb[0], vb[1], obstacle_twist[0], obstacle_twist[1]);
    // ROS_INFO("ASV Heading: %.2f, Obstacle Heading: %.2f, alpha = %.2f",
    //          asv_pose_[2]*RAD2DEG,
    //          obstacle_pose[2]*RAD2DEG,
    //          alpha*RAD2DEG);

    for (int u_it=0; u_it<VEL_SAMPLES_; ++u_it){
      for (int t_it=0; t_it<ANG_SAMPLES_; ++t_it){
        /// @todo
        u = u0 + u_it*du;
        t = theta0 + t_it*dtheta;

        if (inVelocityObstacle(u, t, lb, rb, vb))
          {
            setVelocity(u, t, 100, u_it, t_it);
          }
        else if (violatesColregs(u, t, obstacle_pose, vb))
          {
            setVelocity(u, t, 75, u_it, t_it);
          }
        else
          {
            // @todo This line will cause a bug if multiple obstacles are present.
            setVelocity(u, t, 25, u_it, t_it);
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
  for (int i=0; i<VG_XLIM_*VG_YLIM_; ++i)
    vg_.data[i] = 0;
}

void VelocityObstacle::setVelocity(const double &u, const double &theta, int val, const int &ui, const int &ti)
{
  if (marker_ != NULL)
    {
      // Convert from scalar value to RGB heatmap: https://www.particleincell.com/2014/colormap/
      double val_norm = (double) val;
      // Invert range and group
      val_norm = 4.0 * (100. - val_norm) * 0.01;

      // Get integer part
      double group = floor(val_norm);
      double frac = floor((val_norm - group));

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


      // ROS_INFO("VAL: %.1f, %d, (%.1f, %.1f, %.1f)", val_norm, val, r, g, b);


      marker_->colors[ui*ANG_SAMPLES_ + ti].r = r;
      marker_->colors[ui*ANG_SAMPLES_ + ti].g = g;
      marker_->colors[ui*ANG_SAMPLES_ + ti].b = b;
    }

  setPoint(u*cos(theta-asv_pose_[2]), u*sin(theta-asv_pose_[2]), val);
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

void VelocityObstacle::initializeMarker(visualization_msgs::Marker *marker)
{
  marker_ = marker;
  double u0 = 0, u = 0;
  double theta0 = -MAX_ANG_, t = 0;
  double du = MAX_VEL_/VEL_SAMPLES_;
  double dtheta = 2*MAX_ANG_/ANG_SAMPLES_;

  const double MFACTOR = 10.0;

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
/////// UTILS /////////
void rot2d(const Eigen::Vector2d &v, double yaw, Eigen::Vector2d &result)
{
  Eigen::Matrix2d R;
  R << cos(yaw), -sin(yaw),
    sin(yaw), cos(yaw);
  result = R*v;
}

