#include <bau_local_planner/bau_local_planner_ros.h>
#include <pluginlib/class_list_macros.h>

// registers this class as a move_base plugin we can select for local planning
PLUGINLIB_EXPORT_CLASS(bau_local_planner::BAUPlannerROS, nav_core::BaseLocalPlanner)

namespace bau_local_planner {

BAUPlannerROS::BAUPlannerROS() : initialised_(false), odom_topic_("/odom"), bau_planner(&planner_util_) {}

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
  ROS_INFO("Received new global plan.");
  // reset this controller as it is stateful, and the goal now may have changed
  latched_sr_controller_.resetLatching();

  return planner_util_.setPlan(orig_global_plan);
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
    ROS_ERROR("Failed to get current local plan. Planning failure.");
    return false;
  } else if (local_plan.empty()) {
    ROS_ERROR("Local plan is empty. Planning failure.");
    return false;
  } else {
    ROS_INFO("Sucessfully received a new local plan.");
  }
  // set up the planner with the new goal information we just received
  bau_planner.prePlan(cur_robot_pose_, costmap_->getRobotFootprint(), local_plan);

  if (latched_sr_controller_.isPositionReached(&planner_util_, cur_robot_pose_)) {
    // reach here if the goal has been overshot
    // in which case we want to stop, and potentially rotate towards the goal if
    // we are not oriented yet

  } else {
    geometry_msgs::PoseStamped robot_vel;
    odom_helper_.getRobotVel(robot_vel);
    geometry_msgs::PoseStamped drive_cmds;
    drive_cmds.header.frame_id = costmap_->getBaseFrameID();
    // try to find a new trajectory using the planner
    base_local_planner::Trajectory new_trajectory = bau_planner.plan(cur_robot_pose_, robot_vel, drive_cmds);
    // pass the resulting trajectory on (may be 0, in the case of planning failure)
    cmd_vel.linear.x = drive_cmds.pose.position.x;
    cmd_vel.linear.y = drive_cmds.pose.position.y;
    cmd_vel.angular.z = tf2::getYaw(drive_cmds.pose.orientation);
    if (new_trajectory.cost_ < 0) {
      ROS_ERROR("Unable to find a trajectory, stopping robot.");
      return false;
    }
  }

  return true;
}

bool BAUPlannerROS::isInitialized() { return initialised_; }

}  // namespace bau_local_planner
   // bau_local_planner
