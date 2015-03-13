#ifndef ASV_CTRL_VO
#define ASV_CTRL_VO

#include <Eigen/Dense>

#include "asv_msgs/State.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "visualization_msgs/Marker.h"

class VelocityObstacle
{
 public:
  VelocityObstacle();
  ~VelocityObstacle();

  void initialize(std::vector<asv_msgs::State> *obstacles);
  void update();
  void updateAsvState(const nav_msgs::Odometry::ConstPtr &msg, const double &u_d, const double &psi_d);
  void initializeMarker(visualization_msgs::Marker *marker);
  int getAngRes() {return ANG_SAMPLES_;};
  int getVelRes() {return VEL_SAMPLES_;};
  void getBestControlInput(double &u_best, double &psi_best);


 private:
  void setVelocity(const int &ui, const int &ti, const double &val);
  void updateVelocityGrid();
  void clearVelocityGrid();

  bool inVelocityObstacle(const double &u,
                          const double &theta,
                          const Eigen::Vector2d &lb,
                          const Eigen::Vector2d &rb,
                          const Eigen::Vector2d &vb);
  bool violatesColregs(const double &u,
                       const double &theta,
                       const Eigen::Vector3d &obstacle_pose,
                       const Eigen::Vector2d &vb);


  const double RADIUS_;
  const double MAX_VEL_;
  const double MAX_ANG_;
  const double GRID_RES_;

  const int EDGE_SAMPLES_;
  const int VEL_SAMPLES_;
  const int ANG_SAMPLES_;

  std::vector<double> vo_grid_;

  Eigen::Vector3d asv_pose_;
  Eigen::Vector3d asv_twist_;
  double u_d_;
  double psi_d_;


  std::vector<asv_msgs::State> *obstacles_;

  // ROS API
  // void obstacleSubCallback(const nav_msgs::Odometry::ConstPtr &msg);

  visualization_msgs::Marker *marker_;
};


#endif
