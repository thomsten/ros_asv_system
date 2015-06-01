#include "ros/ros.h"
#include <ros/console.h>

#include "asv_msgs/State.h"
#include "nav_msgs/OccupancyGrid.h"
#include "visualization_msgs/Marker.h"
#include <tf/transform_datatypes.h>

#include <iostream>
#include <cmath>

#include "asv_ctrl_vo/asv_ctrl_vo.h"
static const double DEG2RAD = M_PI/180.0f;
static const double RAD2DEG = 180.0f/M_PI;

// Utils
void print_situation(const colregs_t &sit);
void get_state_str(std::string &str, const colregs_t &state);
void rot2d(const Eigen::Vector2d &v, double yaw, Eigen::Vector2d &result);
void normalize_angle(double &alpha);
void normalize_angle_diff(double &angle, const double &angle_ref);

VelocityObstacle::VelocityObstacle() : EDGE_SAMPLES_(10),
                                       VEL_SAMPLES_(41),
                                       ANG_SAMPLES_(101),
                                       RADIUS_(10.0),
                                       MAX_VEL_(4.0),
                                       MAX_ANG_(2.0944),
                                       MIN_DIST_(100.0),
                                       D_CPA_MIN_(50.0),
                                       T_CPA_MAX_(60.0)
{
  asv_pose_ = Eigen::Vector3d(0.0, 0.0, 0.0);
  asv_twist_ = Eigen::Vector3d(0.0, 0.0, 0.0);
}

VelocityObstacle::~VelocityObstacle()
{
}

void VelocityObstacle::initialize(std::vector<asv_msgs::State> *obstacles,
                                  nav_msgs::OccupancyGrid *map)
{

  ROS_INFO("Initializing VO-node...");

  vo_grid_.resize(ANG_SAMPLES_*VEL_SAMPLES_);

  /// @todo Remove local_map_! Only used for debugging purposes...
  local_map_.header.frame_id ="map";
  local_map_.info.resolution = 0.78;
  local_map_.info.width  = 1362;
  local_map_.info.height = 942;
  local_map_.info.origin.position.x = -(float)496;
  local_map_.info.origin.position.y = -(float)560;

  local_map_.data.resize(local_map_.info.width*local_map_.info.height);
  ros::NodeHandle n;
  lm_pub = n.advertise<nav_msgs::OccupancyGrid>("localmap", 5);
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

  if (state_list_.size() < obstacles_->size()){
    state_list_.resize(obstacles_->size(), NO_SITUATION);
  }

  double vo_buf[VEL_SAMPLES_*ANG_SAMPLES_];

  double u0 = 0, u = 0;
  double theta0 = -MAX_ANG_ + asv_pose_[2], t = 0;

  double du = MAX_VEL_/VEL_SAMPLES_;
  double dtheta = 2*MAX_ANG_/ANG_SAMPLES_;

  Eigen::Vector2d va;
  rot2d(asv_twist_.head(2), asv_pose_[2], va);
  Eigen::Vector2d va_ref = Eigen::Vector2d(u_d_,
                                           psi_d_);

  std::vector<asv_msgs::State>::iterator it;


  /// @todo Move this to own function, e.g., this->resetVelocityField(...);
  // Objective function value. By minimizing this, an "optimal" velocity may be selected.
  double objval = 0.0, max_objval = -1;

  /// @todo Get weighing parameters from rosparam
  Eigen::Matrix2d Q;
  Q << 1.0, 0,
    0, 1.0;
  Eigen::Vector2d vref = Eigen::Vector2d(u_d_*cos(psi_d_), u_d_*sin(psi_d_));
  Eigen::Vector2d vi;
  for (int u_it=0; u_it<VEL_SAMPLES_; ++u_it) {
    for (int t_it=0; t_it<ANG_SAMPLES_; ++t_it) {
      u = u0 + u_it*du;
      t = theta0 + t_it*dtheta;
      // normalize_angle_diff(t, va_ref[1]);

      vi[0] = u*cos(t) - vref[0];
      vi[1] = u*sin(t) - vref[1];

      objval = vi.transpose()*Q*vi;
      vo_buf[u_it*ANG_SAMPLES_ + t_it] = objval;

      if (objval > max_objval) {
        max_objval = objval;
      }
    }
  }

  // Normalize
  for (int u_it=0; u_it<VEL_SAMPLES_; ++u_it) {
    for (int t_it=0; t_it<ANG_SAMPLES_; ++t_it) {
      vo_buf[u_it*ANG_SAMPLES_ + t_it] *= 0.5 / max_objval;
      if (vo_buf[u_it*ANG_SAMPLES_ + t_it] > 1.0)
        {
          ROS_INFO("Objective value: %.2f", objval);
          objval = 1.0;
        }
      setVelocity(u_it, t_it, vo_buf[u_it*ANG_SAMPLES_ + t_it]);
    }
  }

  int i = 0;
  for (it = obstacles_->begin(); it != obstacles_->end(); ++it) {
    Eigen::Vector3d obstacle_pose = Eigen::Vector3d(it->x, it->y, it->psi);
    Eigen::Vector3d obstacle_twist = Eigen::Vector3d(it->u, it->v, it->r);

    double combined_radius = RADIUS_ + it->header.radius;

    Eigen::Vector2d vb;
    rot2d(obstacle_twist.head(2), obstacle_pose[2], vb);

    Eigen::Vector2d pab = -asv_pose_.head(2) + obstacle_pose.head(2);

    // Relative bearing (Loe, 2008)
    double bearing = atan2(-pab[1], -pab[0]) - obstacle_pose[2];

    // Normalize the angle [0, 2*PI)
    while (bearing <= 0)
      bearing += 2*M_PI;
    while(bearing > 2*M_PI)
      bearing -= 2*M_PI;

    double angle_diff = asv_pose_[2] - obstacle_pose[2];
    normalize_angle_diff(angle_diff, asv_pose_[2]);

    bool collision_situation    = inCollisionSituation(asv_pose_, obstacle_pose, va, vb);
    colregs_t colregs_situation = inColregsSituation(bearing, angle_diff);

    bool apply_colregs = (colregs_situation == HEAD_ON ||
                          colregs_situation == CROSSING_RIGHT ||
                          colregs_situation == OVERTAKING) && collision_situation;



    /// @todo Move this to own function, e.g., this->handleEvent(...);
    colregs_t prev_state = state_list_[i];

    switch (state_list_[i])
      {
      case NO_SITUATION:
        if (apply_colregs)
          state_list_[i] = colregs_situation;
        else
          state_list_[i] = NO_SITUATION;
        break;
      case HEAD_ON:
        // Head on is resolved when the colregs_situation changes (relative
        // bearing larger than +/- 112.5 deg)
        if (apply_colregs)
          state_list_[i] = colregs_situation;
        else
          state_list_[i] = NO_SITUATION;
        break;
      case CROSSING_LEFT:
        //
        if (apply_colregs)
          state_list_[i] = colregs_situation;
        else
          state_list_[i] = NO_SITUATION;
        break;
      case CROSSING_RIGHT:
        // Pass behind the other vessel
        // 112.5*DEG2RAD <= alpha < 247.5*DEG2RAD
        if ( !(112.15*DEG2RAD <= bearing  && bearing < 247.5*DEG2RAD) ) {
          collision_situation = true;
          apply_colregs = true;
        }
        else {
          if (apply_colregs) {
            state_list_[i] = colregs_situation;
          }
          else
            state_list_[i] = NO_SITUATION;
        }
      case OVERTAKING:
        // The overtaking situation is maintained until the relative bearing is
        // within +/- 15 deg
        if (colregs_situation != HEAD_ON) {
          collision_situation = true;
          apply_colregs = true;
        }
        else {
          if (apply_colregs)
            state_list_[i] = colregs_situation;
          else
            state_list_[i] = NO_SITUATION;
        }
        break;
      }


    if (state_list_[i] != prev_state) {
      std::string pstate, nstate;
      get_state_str(pstate, prev_state);
      get_state_str(nstate, state_list_[i]);

      ROS_INFO("Ship %d: In %s state, switching to %s", i, pstate.c_str(), nstate.c_str());
    }

    // Collsion situation detected
    // Find velocity obstacle region
    double pab_norm = pab.norm(), alpha = 0.5*M_PI;
    if (pab_norm >= combined_radius)
      alpha = asin(combined_radius / pab_norm);

    // Left and right bounds pointing inwards to the VO. (Guy et. al. 2009)
    Eigen::Vector2d lb, rb;
    pab = pab/pab_norm;
    rot2d(pab,  alpha - 0.5*M_PI, lb);
    rot2d(pab, -alpha + 0.5*M_PI, rb);

    for (int u_it=0; u_it<VEL_SAMPLES_; ++u_it) {
      for (int t_it=0; t_it<ANG_SAMPLES_; ++t_it) {
        // /// @todo
        u = u0 + u_it*du;
        t = theta0 + t_it*dtheta;
        normalize_angle_diff(t, va_ref[1]);

        objval = vo_buf[u_it*ANG_SAMPLES_ + t_it];

        if (collision_situation && inVelocityObstacle(u, t, lb, rb, vb))
          {
            setVelocity(u_it, t_it, VELOCITY_NOT_OK);
          }
        else if ((apply_colregs &&
                  violatesColregs(u, t, obstacle_pose, vb)))
          {
            setVelocity(u_it, t_it, VELOCITY_VIOLATES_COLREGS + objval/2.0);
          }
        else
          {
            /// ALREADY SET
            // setVelocity(u_it, t_it, objval);
          }
      }
    }
    ++i;
  }
}

void VelocityObstacle::checkStaticObstacles()
{
  for (int i=0;i < local_map_.data.size(); ++i)
    local_map_.data[i] = 0;

  if (map_ == NULL || map_->info.resolution <= 0.0)
    return;

  double px0  = asv_pose_[0];
  double py0  = asv_pose_[1];
  double theta0 = asv_pose_[2];

  double u     =  MAX_VEL_;
  double theta = -MAX_ANG_ + theta0, theta_max = MAX_ANG_ + theta0;

  double du     = -MAX_VEL_/VEL_SAMPLES_;
  double dtheta = 2.0*MAX_ANG_/ANG_SAMPLES_;
  double dt     = map_->info.resolution / MAX_VEL_; // Proportional to the grid resolution

  double px, py, dx, dy, t;

  bool velocity_ok;

  // The time limit for static obstacles.
  // Assuming u_d = 3.0 m/s, tlim = 40.0 s => safety_region 120 m (head on)
  double t_max = 40.0;

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
      t = dt;
      velocity_ok = true;
      while (t <= t_max) {
        px = px0 + u*dx*t;
        py = py0 + u*dy*t;

        // THIS MAY CAUSE SEGFAULT :S (Only used for visualization purposes)
        // int px_i = (int) round((px - local_map_.info.origin.position.x)/local_map_.info.resolution);
        // int py_i = (int) round((py - local_map_.info.origin.position.y)/local_map_.info.resolution);
        // local_map_.data[px_i + local_map_.info.width*py_i] = 100;

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

  lm_pub.publish(local_map_);
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
  return map_->data[px_i + py_i*map_->info.width] > OCCUPIED_TRESH;
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

  // COLREGs applicaple if the following relation holds (Kuwata et. al., 2014)
  return ((pdiff[0]*vdiff[1] - pdiff[1]*vdiff[0] < 0.0) &&
          (pdiff.dot(vdiff) < 0));
}

void VelocityObstacle::clearVelocityGrid()
{
  for (int i=0; i<vo_grid_.size(); ++i)
    vo_grid_[i] = 0.0;
}

void VelocityObstacle::setVelocity(const int &ui, const int &ti, const double &val)
{
  int size = (ui*ANG_SAMPLES_ + ti);
  if ( ! (size < vo_grid_.size()) ) {
    ROS_ERROR("Tried to set %d,%d, size: %d max_size: %d", ui, ti, size, (int)vo_grid_.size());
    return;
  }

  // Already sampled with higher priority (e.g. for another obstacle)
  // The value 1337 is used for marking the current best (u, psi) pair
  if (vo_grid_[ui*ANG_SAMPLES_ + ti] >= val && val != 1337.0)
    return;

  // Set colors
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

bool VelocityObstacle::inCollisionSituation(const Eigen::Vector3d &pose_a,
                                            const Eigen::Vector3d &pose_b,
                                            const Eigen::Vector2d &va,
                                            const Eigen::Vector2d &vb)
{
  // Vector from obstacle position to asv position
  Eigen::Vector2d pab = pose_a.head(2) - pose_b.head(2);


  double vab_norm = (va-vb).norm();
  double t_cpa = 0.0;
  if (vab_norm > 0.0001)
    {
      t_cpa = -pab.dot(va-vb)/(vab_norm*vab_norm);
    }

  double d_cpa = ((pose_a.head(2) + va*t_cpa) -
                  (pose_b.head(2) + vb*t_cpa)).norm();

  return ((d_cpa < D_CPA_MIN_) &&
          (t_cpa >= 0.0 && t_cpa < T_CPA_MAX_));
}

colregs_t VelocityObstacle::inColregsSituation(const double &alpha,
                                               const double &angle_diff)
{

  // The limits are found in Loe, 2008.
  if ((0.0 <= alpha && alpha < 15.0*DEG2RAD) ||
      (345.0*DEG2RAD <= alpha && alpha < 360.0*DEG2RAD))
    {
      return HEAD_ON;
    }
  else if (15.0*DEG2RAD <= alpha && alpha < 112.5*DEG2RAD)
    {
      // Crossing from right
      return CROSSING_RIGHT;
    }
  else if (247.5*DEG2RAD <= alpha && alpha < 345.0*DEG2RAD)
    {
      // Crossing from left: No COLREGs
      return CROSSING_LEFT;
    }
  else
    {
      // The remaining: Overtaking
      // 112.5*DEG2RAD <= alpha < 247.5*DEG2RAD
      if (fabs(angle_diff) <= 45*DEG2RAD)
        return OVERTAKING;
      else
        return NO_SITUATION;
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

void normalize_angle(double &alpha)
{
  while (alpha <= -M_PI)
    alpha += 2*M_PI;
  while(alpha > M_PI)
    alpha -= 2*M_PI;
}

void normalize_angle_diff(double &angle, const double &angle_ref)
{
  double diff = angle_ref - angle;

  if (isinf(angle) || isinf(angle_ref))
    return;

  // Get angle within 2PI of angle_ref
  if (diff > 0)
    {
      angle = angle + (diff - fmod(diff, 2*M_PI));
    }
  else
    {
      angle = angle + (diff + fmod(-diff, 2*M_PI));
    }

  // Make sure angle is on the closest side of angle_ref
  diff = angle_ref - angle;
  if (diff > M_PI)
    {
      angle += 2*M_PI;
    }
  else if (diff < -M_PI)
    {
      angle -= 2*M_PI;
    }
}

void get_state_str(std::string &str, const colregs_t &state) {
  switch(state){
  case NO_SITUATION:
    str = std::string("NO_SITUATION");
    break;
  case HEAD_ON:
    str = std::string("HEAD_ON");
    break;
  case CROSSING_LEFT:
    str = std::string("CROSSING_LEFT");
    break;
  case CROSSING_RIGHT:
    str = std::string("CROSSING_RIGHT");
    break;
  case OVERTAKING:
    str = std::string("OVERTAKING");
    break;
  default:
    break;
  }
}

void print_situation(const colregs_t &sit) {
  switch(sit){
  case NO_SITUATION:
    ROS_INFO("In situation: NO_SITUATION");
    break;
  case HEAD_ON:
    ROS_INFO("In situation: HEAD_ON");
    break;
  case CROSSING_LEFT:
    ROS_INFO("In situation: CROSSING_LEFT");
    break;
  case CROSSING_RIGHT:
    ROS_INFO("In situation: CROSSING_RIGHT");
    break;
  case OVERTAKING:
    ROS_INFO("In situation: OVERTAKING");
    break;
  default:
    ROS_INFO("wtf: %d, %d", sit, OVERTAKING);
    break;
  }
}
