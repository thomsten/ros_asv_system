#include "ros/ros.h"
#include <ros/console.h>
#include <pluginlib/class_list_macros.h>

#include <vector>

#include "nav_msgs/OccupancyGrid.h"
#include "asv_msgs/StateArray.h"
#include "visualization_msgs/Marker.h"

#include "asv_ctrl_vo_plugin/asv_ctrl_vo_plugin.h"
#include "asv_ctrl_vo_plugin/asv_ctrl_vo_plugin_node.h"


// Register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(asv_ctrl_vo::VelocityObstacleNode, nav_core::BaseLocalPlanner)

namespace asv_ctrl_vo
{

  VelocityObstcleNode::VelocityObstacleNode : initialized_(false) {};


  void VelocityObstacleNode::initialize(std::string name,
                                        tf::TransformListener *tf
                                        costmap_2d::Costmap2DROS *costmap_ros) {
    if (! isInitialized())
      {
        ros::NodeHandle nh;

        vo_ = new VelocityObstacle;

        mk_pub_  = n.advertise<visualization_msgs::Marker>("vo_markers", 1);

        obstacle_sub_ = nh.subscribe("obstacle_states",
                                     1,
                                     obstacleCallback,
                                     this);
        asv_sub_ = nh.subscribe("asv/state",
                                1,
                                asvCallback,
                                this);

        costmap_ros_ = costmap_ros;

        vo_->initialize(&dynamic_obstacles_, costmap_ros);
        initializeMarkers();

        initialized_ = true;

      }
    else
      {
        ROS_WARN("This planner has already been initialized, doing nothing.")
      }
  }


  bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel) {

    vo_->update();
    vo_->getBestControlInput(asv_pose_, asv_twist_, cmd_vel);

    /// @todo This should probably be placed elsewhere. Meeeeh..
    marker_.header.stamp = ros::Time();
    mk_pub_->publish(marker_);

    /// @todo Returns true if a valid velocity command is found. Now we assume
    /// this is always the case.
    return true;
  }

  bool isGoalReached() {
    return vo_->isGoalReached()
  }

  bool setPlan(const std::vector<geometry_msgs::PoseStamped> &plan) {
    return vo_->setPlan(plan);
  }
}



void VelocityObstacleNode::asvCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  vo_->updateAsvState(msg, u_d_, psi_d_);
}

void VelocityObstacleNode::obstacleCallback(const asv_msgs::StateArray::ConstPtr &msg)
{
  obstacles_ = msg->states;
}

void VelocityObstacleNode::initializeMarkers()
{
  // Setup marker message
  marker_.header.stamp = ros::Time::now();
  marker_.header.frame_id = std::string("asv");
  marker_.ns = std::string("asv");
  marker_.type = visualization_msgs::Marker::POINTS;
  marker_.action = visualization_msgs::Marker::ADD;
  //marker_.pose = geometry_msgs::Pose();
  marker_.scale.x = 0.5;
  marker_.scale.y = 0.5;


  vo_->initializeMarker(&marker_);
}
