#ifndef ASV_CTRL_VO
#define ASV_CTRL_VO

#include <Eigen/Dense>

#include "asv_msgs/State.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"

class VelocityObstacle
{
 public:
  VelocityObstacle();
  ~VelocityObstacle();

  void initialize(std::vector<asv_msgs::State> *obstacles);
  void update(const ros::Publisher *og_pub);
  void updateAsvState(const nav_msgs::Odometry::ConstPtr &msg);

 private:

  void setPoint(const double &px, const double &py, int val);
  void setVelocity(const double &u, const double &theta, int val);
  void updateVelocityGrid();
  void clearVelocityGrid();

  bool inVelocityObstacle(const double &u,
                          const double &theta,
                          const Eigen::Vector2d &lb,
                          const Eigen::Vector2d &rb,
                          const Eigen::Vector3d &obstacle_twist);
  bool violatesColregs(const double &u,
                       const double &theta,
                       const Eigen::Vector3d &obstacle_pose,
                       const Eigen::Vector3d &obstacle_twist);


  const double RADIUS_;
  const double MAX_VEL_;
  const double MAX_ANG_;
  const double GRID_RES_;

  const int EDGE_SAMPLES_;
  const int VEL_SAMPLES_;
  const int ANG_SAMPLES_;

  Eigen::Vector3d asv_pose_;
  Eigen::Vector3d asv_twist_;

  int VG_XLIM_;
  int VG_YLIM_;

  std::vector<asv_msgs::State> *obstacles_;

  // ROS API
  // void obstacleSubCallback(const nav_msgs::Odometry::ConstPtr &msg);

  nav_msgs::OccupancyGrid vg_;
};


#endif
