#include "ros/ros.h"
#include <ros/console.h>

#include "asv_msgs/State.h"
#include "nav_msgs/OccupancyGrid.h"
#include "visualization_msgs/Marker.h"
#include <tf/transform_datatypes.h>

#include <iostream>
#include <cmath>

#include "asv_ctrl_vo/asv_ctrl_vo.h"


void rot2d(const Eigen::Vector2d &v, double yaw, Eigen::Vector2d &result);
void normalize_angle(double &angle);

VelocityObstacle::VelocityObstacle() : EDGE_SAMPLES_(10),
                                       VEL_SAMPLES_(41),
                                       ANG_SAMPLES_(101),
                                       RADIUS_(10.0),
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

void VelocityObstacle::initialize(std::vector<asv_msgs::State> *obstacles, nav_msgs::OccupancyGrid *map)
{

  ROS_INFO("Initializing VO-node...");

  vo_grid_.resize(ANG_SAMPLES_*VEL_SAMPLES_);

  obstacles_ = obstacles;
  map_ = map;

  ROS_INFO("Initialization complete!");
}

void VelocityObstacle::update()
{
  clearVelocityGrid();
  checkStaticObstacles();
  updateVelocityGrid();
}

void VelocityObstacle::updateVelocityGrid()
{
  const double RAD2DEG = 180.0/M_PI;
  const double OBJVAL_SCALE = 11.0;

  double u0 = 0, u = 0;
  double theta0 = -MAX_ANG_ + asv_pose_[2], t = 0;
  double du = MAX_VEL_/VEL_SAMPLES_;
  double dtheta = 2*MAX_ANG_/ANG_SAMPLES_;

  Eigen::Vector2d va;
  rot2d(asv_twist_.head(2), asv_pose_[2], va);
  Eigen::Vector2d va_ref = Eigen::Vector2d(u_d_,
                                           psi_d_);

  std::vector<asv_msgs::State>::iterator it;

  double T_CPA_MAX = 100.0, D_CPA_MIN = 100.0;

  double uerr, terr;

  // Objective function value. By minimizing this, an "optimal" velocity may be selected.
  double objval = 0.0;

  if (obstacles_->empty())
    {
      for (int u_it=0; u_it<VEL_SAMPLES_; ++u_it) {
        for (int t_it=0; t_it<ANG_SAMPLES_; ++t_it) {
          /// @todo
          u = u0 + u_it*du;
          t = theta0 + t_it*dtheta;

          uerr = fabs(u - va_ref[0]);
          terr = fabs(t - va_ref[1]);

          objval = (0.05*uerr*uerr + 0.95*terr*terr) / OBJVAL_SCALE;

          setVelocity(u_it, t_it, objval);
        }
      }
    }

  for (it = obstacles_->begin(); it != obstacles_->end(); ++it) {
    Eigen::Vector3d obstacle_pose = Eigen::Vector3d(it->x, it->y, it->psi);
    Eigen::Vector3d obstacle_twist = Eigen::Vector3d(it->u, it->v, it->r);

    // Vector from obstacle position to asv position
    Eigen::Vector2d pab = -asv_pose_.head(2) + obstacle_pose.head(2);

    Eigen::Vector2d vb;
    rot2d(obstacle_twist.head(2), obstacle_pose[2], vb);

    double vab_norm = (vb-va).norm();
    double t_cpa = 0.0;
    if (vab_norm > 0.0001)
      {
        t_cpa = -pab.dot(vb-va)/(vab_norm*vab_norm);
      }

    double d_cpa = ((asv_pose_.head(2) + va*t_cpa) -
                    (obstacle_pose.head(2) + vb*t_cpa)).norm();


    bool collision_situation = false;

    if ((d_cpa < D_CPA_MIN) &&
        (t_cpa >= 0.0 && t_cpa < T_CPA_MAX))
      {
        collision_situation = true;
      }

    // COLREGs situation detected!
    // Find velocity obstacle region
    double alpha = asin(2*RADIUS_ / pab.norm());

    // Left and right bounds pointing inwards to the VO. (Guy et. al. 2009)
    Eigen::Vector2d lb, rb;
    pab = pab/pab.norm();
    rot2d(pab,  alpha - 0.5*M_PI, lb);
    rot2d(pab, -alpha + 0.5*M_PI, rb);
    bool colregs = false;
    for (int u_it=0; u_it<VEL_SAMPLES_; ++u_it) {
      for (int t_it=0; t_it<ANG_SAMPLES_; ++t_it) {
        /// @todo
        u = u0 + u_it*du;
        t = theta0 + t_it*dtheta;

        uerr = fabs(u - va_ref[0]);
        terr = fabs(t - va_ref[1]);

        objval = (0.1*uerr*uerr + 0.85*terr*terr) / OBJVAL_SCALE;

        if (collision_situation && inVelocityObstacle(u, t, lb, rb, vb))
          {
            setVelocity(u_it, t_it, VELOCITY_NOT_OK);
          }
        else if (collision_situation && violatesColregs(u, t, obstacle_pose, vb))
          {
            setVelocity(u_it, t_it, VELOCITY_VIOLATES_COLREGS + objval/2.0);
            colregs = true;
          }
        else
          {
            /// @todo This line will cause a bug if multiple obstacles are present.
            /// EDIT: Solved in setVelocity by checking if value already present in
            /// the VO-grid is larger than the value to be added. Should this be
            /// Tested here instead?
            setVelocity(u_it, t_it, objval);
          }
      }
    }
    if (colregs)
      ROS_INFO("Changed: tcpa,dcpa: (%.2f,%.2f)", t_cpa, d_cpa);

  }
}

void VelocityObstacle::checkStaticObstacles()
{
  if (map_ == NULL || map_->info.resolution <= 0.0)
    return;
  const double RAD2DEG = 180.0/M_PI;

  double u = MAX_VEL_, u_min = 0.0;
  double theta = -MAX_ANG_ + asv_pose_[2], theta_max = MAX_ANG_ + asv_pose_[2];

  double du = -MAX_VEL_/VEL_SAMPLES_;
  double dtheta = 2*MAX_ANG_/ANG_SAMPLES_;

  // The time limit for static obstacles.
  // Assuming u_d = 3.0 m/s, tlim = 40.0 s => safety_region 120 m (head on)
  double t_max = 40.0;

  double dt = map_->info.resolution / MAX_VEL_; /// @todo This size is proportional to the grid size and velocity

  double t = 0;

  double px, py, dx, dy;
  double px0 = asv_pose_[0], py0 = asv_pose_[1];

  bool velocity_ok;
  /// Note that we loop through the velocity in decreasing order because if the
  /// largest velocity (-path) is collision free, so will the smaller ones be as
  /// well.
  for (int theta_it = 0; theta_it < ANG_SAMPLES_; ++theta_it) {

    // Reset u
    u = MAX_VEL_;
    dx = cos(theta);
    dy = sin(theta);
    for (int u_it = VEL_SAMPLES_-1; u_it >= 0; --u_it) {
      // Reset t
      t = 0;
      velocity_ok = true;
      while (t <= t_max) {
        px = px0 + u*dx*t;
        py = py0 + u*dy*t;

        if (inObstacle(px, py))
          {
            velocity_ok = false;
            break;
          }
        t += dt;
      }

      if (velocity_ok)
        {
          // This direction is ok
          // Mark all velocities from here as ok!
          for (int i=u_it; i >= 0; --i)
            setVelocity(i, theta_it, VELOCITY_OK);
          break;
        }
      else
        {
          setVelocity(u_it, theta_it, VELOCITY_NOT_OK);
        }

      u += du;
    }

    theta += dtheta;
  }

}


bool VelocityObstacle::inVelocityObstacle(const double &u,
                                          const double &theta,
                                          const Eigen::Vector2d &lb,
                                          const Eigen::Vector2d &rb,
                                          const Eigen::Vector2d &vb)
{
  Eigen::Vector2d va(u*cos(theta), u*sin(theta));

  return ((va-vb).dot(lb) >= 0.0 && (va-vb).dot(rb) >= 0.0);
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
  // Normalize the angle [0, 2*PI)
  while (angle <= 0)
    angle += 2*M_PI;
  while(angle > 2*M_PI)
    angle -= 2*M_PI;


  const double DEG2RAD = M_PI/180.0f;

  // The limits are found in Loe, 2008.
  if ((0.0 <= alpha && alpha < 15.0*DEG2RAD) ||
      (345.0*DEG2RAD <= alpha && alpha < 360.0*DEG2RAD))
    {
      // Head-on: COLREGs applicaple if the following relation holds (Kuwata et. al., 2014)
      return (pdiff[0]*vdiff[1] - pdiff[1]*vdiff[0] < 0.0);
    }
  else if (15.0*DEG2RAD <= alpha && alpha < 112.5*DEG2RAD)
    {
      // Crossing from right
      return (pdiff[0]*vdiff[1] - pdiff[1]*vdiff[0] < 0.0);
    }
  else if (247.5*DEG2RAD <= alpha && alpha < 345.0*DEG2RAD)
    {
      // Crossing from left: No COLREGs
      return false;
    }
  else
    {
      // The remaining: Overtaking
      // 112.5*DEG2RAD <= alpha or alpha < 247.5*DEG2RAD
      // Passing on both sides allowed.
      return false; //(pdiff[0]*vdiff[1] - pdiff[1]*vdiff[0] <= 0.0);
    }
}

void VelocityObstacle::clearVelocityGrid()
{
  for (int i=0; i<vo_grid_.size(); ++i)
    vo_grid_[i] = 0.0;
}

void VelocityObstacle::setVelocity(const int &ui, const int &ti, const double &val)
{
  // Already sampled with higher priority (e.g. for another obstacle)
  if (vo_grid_[ui*ANG_SAMPLES_ + ti] >= val && val != 1337.0)
    return;


  if (marker_ != NULL)
    {
      // Convert from scalar value to RGB heatmap: https://www.particleincell.com/2014/colormap/
      double newval = 4*(1.0 - val);

      if (val == 1337.0) {
        marker_->colors[ui*ANG_SAMPLES_ + ti].r = 0;
        marker_->colors[ui*ANG_SAMPLES_ + ti].g = 0;
        marker_->colors[ui*ANG_SAMPLES_ + ti].b = 0;
      } else {

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
        marker_->colors[ui*ANG_SAMPLES_ + ti].g = b;
        marker_->colors[ui*ANG_SAMPLES_ + ti].b = g;
      }
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

  marker_->points.resize(VEL_SAMPLES_*ANG_SAMPLES_);
  marker_->colors.resize(VEL_SAMPLES_*ANG_SAMPLES_);

  /// The multiplication factor (spreading of points / size of visual VO-field)
  /// Or, the magic factor if you like...
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

  setVelocity(ui, ti, 1337.0);

  double du = MAX_VEL_/VEL_SAMPLES_;
  double dtheta = 2*MAX_ANG_/ANG_SAMPLES_;


  u_best = ui*du;
  psi_best = -MAX_ANG_ + ti*dtheta + asv_pose_[2];
}

bool VelocityObstacle::inObstacle(double px, double py)
{
  // Assume the map isn't rotated.
  px -= map_->info.origin.position.x;
  py -= map_->info.origin.position.y;

  int px_i = (int) round(px / map_->info.resolution);
  int py_i = (int) round(py / map_->info.resolution);


  if ((px_i < 0 || px_i >= map_->info.width ||
       py_i < 0 || py_i >= map_->info.height))
    {
      return false;
    }

  return map_->data[px_i + py_i*map_->info.height] > OCCUPIED_TRESH;
}

/////// UTILS /////////
void rot2d(const Eigen::Vector2d &v, double yaw, Eigen::Vector2d &result)
{
  Eigen::Matrix2d R;
  R << cos(yaw), -sin(yaw),
    sin(yaw), cos(yaw);
  result = R*v;
}
