#include <bau_local_planner/bau_local_planner_ros.h>
#include <pluginlib/class_list_macros.h>

// registers this class as a move_base plugin we can select for local planning
PLUGINLIB_EXPORT_CLASS(bau_local_planner::BAUPlannerROS, nav_core::BaseLocalPlanner)

namespace bau_local_planner {

BAUPlannerROS::BAUPlannerROS() : initialised_(false), odom_topic_("/odom") {}

void BAUPlannerROS::initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap) {
  ROS_INFO("Attempting to initialize BAUPlanner.");
  if (isInitialized()) {
    ROS_ERROR("BAUPlanner already initialized.");
    return;
  }
  ros::NodeHandle private_nh("~/" + name);
  tf_ = tf;
  costmap_ = costmap;
  costmap_->getRobotPose(cur_robot_pose_);
  odom_helper_.setOdomTopic(odom_topic_);

  initialised_ = true;
}

bool BAUPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan) {
  if (!isInitialized()) {
    ROS_ERROR("BAUPlanner is not yet initialized.");
    return false;
  }

  return true;
}

bool BAUPlannerROS::isGoalReached() {
  if (!isInitialized()) {
    ROS_ERROR("BAUPlanner is not yet initialized.");
    return false;
  }
  // first see if we can get the current robot pose
  if (!costmap_->getRobotPose(cur_robot_pose_)) {
    ROS_ERROR("Unable to get current robot pose.");
    return false;
  }
  // if so, pass it on to the motion controller
  if (latched_sr_controller_.isGoalReached(&planner_util_, odom_helper_, cur_robot_pose_)) {
    ROS_INFO("Goal has been reached.");
    return true;
  }
  return false;
}

bool BAUPlannerROS::computeVelocityCommands(geometry_msgs::Twist &cmd_vel) {
  if (!costmap_->getRobotPose(cur_robot_pose_)) {
    ROS_ERROR("Unable to get current robot pose.");
    return false;
  }
  // first get the global plan translated into our local frame
  std::vector<geometry_msgs::PoseStamped> local_plan;
  if (!planner_util_.getLocalPlan(cur_robot_pose_, local_plan)) {
    ROS_ERROR("Failed to get current plan.");
    return false;
  }
  // TODO: update plan here

  return true;
}

bool BAUPlannerROS::isInitialized() { return initialised_; }

}  // namespace bau_local_planner
   // bau_local_planner
