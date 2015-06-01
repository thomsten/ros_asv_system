#ifndef ASV_CTRL_VO
#define ASV_CTRL_VO

#include <Eigen/Dense>

#include "asv_msgs/State.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "visualization_msgs/Marker.h"

static const double VELOCITY_OK = 0.0;
static const double VELOCITY_VIOLATES_COLREGS = 0.75;
static const double VELOCITY_NOT_OK = 1.0;
static const int OCCUPIED_TRESH = 40;

typedef enum {
  NO_SITUATION=0, // 0
  HEAD_ON,        // 1
  CROSSING_LEFT,  // 2
  CROSSING_RIGHT, // 3
  OVERTAKING      // 4
} colregs_t;

class VelocityObstacle
{
 public:
  /// Constructor
  VelocityObstacle();
  /// Destructor
  ~VelocityObstacle();

  /**
   * @brief Initializes the controller.
   *
   * @param obstacles A pointer to a vector of obstacles provided by the ROS 
   * wrapper (by subscribing to an obstacle tracker node).
   * @param map A pointer to the occupancy grid published by the map_server.
   */
  void initialize(std::vector<asv_msgs::State> *obstacles, nav_msgs::OccupancyGrid *map);
  /**
   * @brief Updates the Velocity Obstacle.
   */
  void update();
  /**
   * @brief Callback for updating the internal ASV state (data provided by ROS wrapper).
   *
   * @param msg The Odometry message which contains the state data.
   * @param u_d The desired surge speed set point provided by, e.g., a LOS algorithm.
   * @param psi_d The desired heading set point provided by, e.g., a LOS algorithm.
   */
  void updateAsvState(const nav_msgs::Odometry::ConstPtr &msg, 
                      const double &u_d, 
                      const double &psi_d);
  /**
   * Initializes the visualization markers used to display the velocity field.
   *
   * @param marker A pointer to a marker message (provided by the ROS wrapper).
   */
  void initializeMarker(visualization_msgs::Marker *marker);

  /**
   * @brief Called after the velocity field has been updated to get the (u, psi) pair
   * with the lowest cost.
   *
   * @param u_best The reference parameter to store the "best" surge speed.
   * @param psi_best The reference parameter to store the "best" heading.
   */
  void getBestControlInput(double &u_best, double &psi_best);
  
 private:
  void setVelocity(const int &ui, const int &ti, const double &val);
  void updateVelocityGrid();
  void clearVelocityGrid();
  void checkStaticObstacles();
  
  bool inVelocityObstacle(const double &u,
                          const double &theta,
                          const Eigen::Vector2d &lb,
                          const Eigen::Vector2d &rb,
                          const Eigen::Vector2d &vb);
  bool violatesColregs(const double &u,
                       const double &theta,
                       const Eigen::Vector3d &obstacle_pose,
                       const Eigen::Vector2d &vb);
  
  bool inObstacle(double px, double py);

  bool inCollisionSituation(const Eigen::Vector3d &pose_a,
                          const Eigen::Vector3d &pose_b,
                          const Eigen::Vector2d &va,
                          const Eigen::Vector2d &vb);
  colregs_t inColregsSituation(const double &bearing,
                               const double &angle_diff);

  std::vector<colregs_t> state_list_;

  const double RADIUS_;
  const double MAX_VEL_;
  const double MAX_ANG_;
  const double MIN_DIST_;

  const double D_CPA_MIN_;
  const double T_CPA_MAX_;

  const int EDGE_SAMPLES_;
  const int VEL_SAMPLES_;
  const int ANG_SAMPLES_;

  std::vector<double> vo_grid_;

  Eigen::Vector3d asv_pose_;
  Eigen::Vector3d asv_twist_;
  double u_d_;
  double psi_d_;

  

  // ROS API
  // void obstacleSubCallback(const nav_msgs::Odometry::ConstPtr &msg);
  std::vector<asv_msgs::State> *obstacles_;
  nav_msgs::OccupancyGrid *map_;
  visualization_msgs::Marker *marker_;
  nav_msgs::OccupancyGrid local_map_;
  ros::Publisher lm_pub;
};


#endif
