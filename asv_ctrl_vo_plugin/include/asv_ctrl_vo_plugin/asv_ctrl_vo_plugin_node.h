#ifndef ASV_CTRL_VO_PLUGIN_NODE
#define ASV_CTRL_VO_PLUGIN_NODE

#include "asv_msgs/StateArray.h"
#include "nav_msgs/Odometry.h"
#include "visualization_msgs/Marker.h"

#include <vector>

namespace asv_ctrl_vo {

  class VelocityObstacleNode : public nav_core::BaseLocalPlanner
  {
  public:
    /// Constructor for VO ROS wrapper
    VelocityObstacleNode();
    /// Destructor for VO ROS wrapper
    ~VelocityObstacleNode();

    void initialize(std::string name,
                    tf::TransformListener *tf,
                    costmap_2d::Costmap2DROS *costmap_ros);

    bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);

    bool isGoalReached();

    bool setPlan(const std::vector<geometry_msgs::PoseStamped> &plan);

  private:

    void asvCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void obstacleCallback(const asv_msgs::StateArray::ConstPtr &msg);

    void initializeMarkers();

    VelocityObstacle *vo_;

    visualization_msgs::Marker marker_;

    Eigen::Vector3d asv_pose_;
    Eigen::Vector3d asv_twist_;

    std::vector<asv_msgs::State> obstacles_;

    base_local_planner::OdometryHelperRos odom_helper_;

    // ROS API
    ros::Publisher mk_pub_;

    ros::Subscriber obstacle_sub_;
    ros::Subscriber asv_sub_;
  };

}
#endif
