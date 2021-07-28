#pragma once
#include <base_local_planner/latched_stop_rotate_controller.h>
#include <base_local_planner/local_planner_util.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <bau_local_planner/bau_local_planner.h>
#include <nav_core/base_local_planner.h>

#include "std_msgs/Bool.h"

namespace bau_local_planner {
/**
 * @brief The BAUPlannerROS class is the ROS-facing interface to the core planner,
 *        it handles retreiving global path information, launching and calling the
 *        local planner, and sending its results on to other systems (visualisation, control)
 */
class BAUPlannerROS : public nav_core::BaseLocalPlanner {
 public:
  /**
   * @brief Constructor for the ROS wrapper for the BAU local planner.
   */
  BAUPlannerROS();

  /**
   * @brief Initialisation procedure for the ROS wrapper for the BAU local
   * planner.
   * @param name - the name of this ROS node
   * @param tf - a tf buffer
   * @param costmap - costmap provided by the localisation/nav system
   */
  void initialize(const std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap);

  /**
   * @brief Destructor for the ROS wrapper for the BAU local planner.
   */
  ~BAUPlannerROS() {}

  /**
   * @brief Handles calling and sending the results of the local planner to the control system.
   * @param cmd_vel - the next set of robot commands to be sent to the controller
   * @return bool - true if a valid trajectory has been found, false if not
   */
  bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);

  /**
   * @brief Tells us if this ROS wrapper has completed its initialisation sequence
   * @return bool - true if initialisation was successful, false if not
   */
  bool isInitialized();

  /**
   * @brief Tells us if the robot has reached its current (local) goal
   * @return bool - true if it has, false if not
   */
  bool isGoalReached();

  /**
   * @brief Upon receipt of a new global plan, resets controller and updates utils class
   * @return bool - true if able to set new plan, false if not.
   */
  bool setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan);

  void stuckCheckCallback(const ros::TimerEvent &);

 private:
  bool initialised_;                            //!< set to true when the planner has completed initialisation
  costmap_2d::Costmap2DROS *costmap_;           //!< costmap from nav system
  tf2_ros::Buffer *tf_;                         //!< tf buffer for transforming between map frames
  geometry_msgs::PoseStamped cur_robot_pose_;   //!< current pose of our robot according to nav system
  geometry_msgs::PoseStamped last_robot_pose_;  //!< last robot pose, used by stuck check

  base_local_planner::OdometryHelperRos odom_helper_;                      //!< helper class for odom data
  base_local_planner::LocalPlannerUtil planner_util_;                      //!< helper class for local planning
  base_local_planner::LatchedStopRotateController latched_sr_controller_;  //!< basic motion controller
  const std::string stuck_topic_;            //!< topic to publish to if we think the robot is stuck
  const std::string odom_topic_;             //!< topic name where odometry is published
  std::shared_ptr<BAUPlanner> bau_planner_;  //!< the actual core path planner
  ros::Publisher robot_stuck_publisher_;     //!< publisher for the stuck message
  bool has_plan_;
  ros::Timer stuck_check_timer_;  //!< controls when we check to see if the robot is stuck
  float stuck_dist_threshold_;
};

}  // namespace bau_local_planner
