#ifndef ASV_CTRL_VO_NODE
#define ASV_CTRL_VO_NODE

#include "asv_msgs/StateArray.h"
#include "nav_msgs/Odometry.h"
#include <vector>

class VelocityObstacleNode
{
 public:
  VelocityObstacleNode();
  ~VelocityObstacleNode();

  void initialize(ros::Publisher *og_pub,
                  ros::Publisher *cmd_pub,
                  ros::Subscriber *obstacle_sub,
                  ros::Subscriber *asv_sub,
                  VelocityObstacle *vo);
  void start();
  void asvCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void obstacleCallback(const asv_msgs::StateArray::ConstPtr &msg);

 private:

  VelocityObstacle *vo_;

  std::vector<asv_msgs::State> obstacles_;
  Eigen::Vector3d asv_pose_;
  Eigen::Vector3d asv_twist_;


  // ROS API
  ros::Publisher *cmd_pub_;
  ros::Publisher *og_pub_;

  ros::Subscriber *obstacle_sub_;
  ros::Subscriber *asv_sub_;
};


#endif
