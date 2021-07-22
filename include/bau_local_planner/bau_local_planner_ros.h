#pragma once
#include <base_local_planner/latched_stop_rotate_controller.h>
#include <base_local_planner/local_planner_util.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <bau_local_planner/bau_local_planner.h>
#include <nav_core/base_local_planner.h>

namespace bau_local_planner {

class BAUPlannerROS : public nav_core::BaseLocalPlanner {
 public:
  /**
   * @brief Constructor for the ROS wrapper for the BAU local planner.
   */
  BAUPlannerROS();

  /**
   * @brief Initialisation procedure for the ROS wrapper for the BAU local
   * planner.
   */
  void initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap);

  /**
   * @brief Destructor for the ROS wrapper for the BAU local planner.
   */
  ~BAUPlannerROS();

  bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);

  bool isInitialized();

  bool isGoalReached();

  bool setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan);

 private:
  bool initialised_;                                   //!< set to true when the planner has completed initialisation
  costmap_2d::Costmap2DROS *costmap_;                  //!< costmap from nav system
  tf2_ros::Buffer *tf_;                                //!< tf buffer for transforming between map frames
  geometry_msgs::PoseStamped cur_robot_pose_;          //!< current pose of our robot according to nav system
  base_local_planner::OdometryHelperRos odom_helper_;  //!< helper class for odom data
  base_local_planner::LocalPlannerUtil planner_util_;  //!< helper class for local planning
  base_local_planner::LatchedStopRotateController latched_sr_controller_;  //!< basic motion controller
  const std::string odom_topic_;                                           //!< topic name where odometry is published
  BAUPlanner bau_planner;                                                  //!< the actual path planner
};

}  // namespace bau_local_planner
