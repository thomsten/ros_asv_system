#include "ros/ros.h"
#include <ros/console.h>
#include <vector>

#include "nav_msgs/OccupancyGrid.h"
#include "asv_msgs/StateArray.h"

#include "asv_ctrl_vo/asv_ctrl_vo.h"
#include "asv_ctrl_vo/asv_ctrl_vo_node.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "asv_ctrl_vo_node");
  ros::start();

  ROS_INFO("Starting VO-node!");

  ros::NodeHandle n;

  VelocityObstacleNode vo_node;
  VelocityObstacle *vo = new VelocityObstacle;

  ros::Publisher og_pub = n.advertise<nav_msgs::OccupancyGrid>("velocity_obstacle", 10);
  ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  ros::Subscriber obstacle_sub = n.subscribe("obstacle_states",
                                             1,
                                             &VelocityObstacleNode::obstacleCallback,
                                             &vo_node);
  ros::Subscriber asv_sub = n.subscribe("state",
                                        1,
                                        &VelocityObstacleNode::asvCallback,
                                        &vo_node);

  vo_node.initialize(&og_pub, &cmd_pub, &obstacle_sub, &asv_sub, vo);
  vo_node.start();

  ros::shutdown();
  return 0;
}


VelocityObstacleNode::VelocityObstacleNode() : vo_(NULL),
                                               cmd_pub_(NULL),
                                               og_pub_(NULL),
                                               obstacle_sub_(NULL),
                                               asv_sub_(NULL) {};

VelocityObstacleNode::~VelocityObstacleNode() {}

void VelocityObstacleNode::initialize(ros::Publisher *og_pub,
                                      ros::Publisher *cmd_pub,
                                      ros::Subscriber *obstacle_sub,
                                      ros::Subscriber *asv_sub,
                                      VelocityObstacle *vo)
{
  cmd_pub_ = cmd_pub;
  og_pub_ = og_pub;
  obstacle_sub_ = obstacle_sub;
  asv_sub_ = asv_sub;

  vo_ = vo;

  vo_->initialize(&obstacles_);
}

void VelocityObstacleNode::start()
{
  ros::Rate loop_rate(10.0);

  while (ros::ok())
    {
      vo_->update(og_pub_);

      ros::spinOnce();
      loop_rate.sleep();
    }

}


void VelocityObstacleNode::asvCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  vo_->updateAsvState(msg);
}

void VelocityObstacleNode::obstacleCallback(const asv_msgs::StateArray::ConstPtr &msg)
{
  obstacles_ = msg->states;
}
