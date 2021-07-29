#include <bau_local_planner/bau_local_planner_ros.h>
#include <math.h>
#include <pluginlib/class_list_macros.h>

// registers this class as a move_base plugin we can select for local planning
PLUGINLIB_EXPORT_CLASS(bau_local_planner::BAUPlannerROS, nav_core::BaseLocalPlanner)

namespace bau_local_planner {

BAUPlannerROS::BAUPlannerROS()
    : initialised_(false),
      stuck_topic_("/robot_stuck"),
      odom_topic_("/odom"),
      has_plan_(false),
      stuck_dist_threshold_(0.05) {
  ROS_INFO("BAUPlanner Instantiated");
}

void BAUPlannerROS::initialize(const std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap) {
  ROS_INFO("Attempting to initialize BAUPlanner.");
  if (isInitialized()) {
    ROS_ERROR("BAUPlanner already initialized.");
    return;
  }
  ros::NodeHandle nh("~/" + name);
  tf_ = tf;
  costmap_ = costmap;
  costmap_->getRobotPose(cur_robot_pose_);
  odom_helper_.setOdomTopic(odom_topic_);
  planner_util_.initialize(tf, costmap_->getCostmap(), costmap_->getGlobalFrameID());

  base_local_planner::LocalPlannerLimits limits;
  limits.max_vel_trans = 0.22;
  limits.min_vel_trans = 0.11;
  limits.max_vel_x = 0.22;
  limits.min_vel_x = -0.22;
  limits.max_vel_y = 0.0;
  limits.min_vel_y = 0.0;
  limits.max_vel_theta = 2.75;
  limits.min_vel_theta = 1.37;
  limits.acc_lim_x = 2.5;
  limits.acc_lim_y = 0.0;
  limits.acc_lim_theta = 3.2;
  limits.acc_lim_trans = 0.0;
  limits.xy_goal_tolerance = 0.05;
  limits.yaw_goal_tolerance = 0.17;
  limits.prune_plan = true;
  limits.trans_stopped_vel = 10.0;
  limits.theta_stopped_vel = 10.0;
  planner_util_.reconfigureCB(limits, false);

  bau_planner_ = std::make_shared<BAUPlanner>(&planner_util_);
  robot_stuck_publisher_ = nh.advertise<std_msgs::Bool>(stuck_topic_, 10);
  stuck_check_timer_ = nh.createTimer(ros::Duration(3.0), &BAUPlannerROS::stuckCheckCallback, this);
  costmap_->getRobotPose(cur_robot_pose_);
  last_robot_pose_ = cur_robot_pose_;
  initialised_ = true;
}

void BAUPlannerROS::stuckCheckCallback(const ros::TimerEvent &) {
  if (!costmap_->getRobotPose(cur_robot_pose_)) {
    ROS_ERROR("Unable to get current robot pose.");
    return;
  }
  std_msgs::Bool msg;
  msg.data = false;
  // if the robot isn't following a plan, assume it isn't stuck
  if (!has_plan_) {
    robot_stuck_publisher_.publish(msg);
    return;
  }

  // here we just check the euclidian distance between the last known pose and the
  // current pose
  float xd = cur_robot_pose_.pose.position.x - last_robot_pose_.pose.position.x;
  float yd = cur_robot_pose_.pose.position.y - last_robot_pose_.pose.position.y;
  float dist = sqrt(xd * xd + yd * yd);
  // TODO: should also add orientation check, as we may be rotating towards a goal in-place
  ROS_INFO("Robot has traveled %fm since last check.", dist);
  if (dist < stuck_dist_threshold_) {
    ROS_INFO("Robot is likely stuck.");
    msg.data = true;
  }
  robot_stuck_publisher_.publish(msg);
  // next time, compare to the current pose
  last_robot_pose_ = cur_robot_pose_;
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
  // if so, pass it on to the motion controller and see if we're at the desired x,y,θ
  if (latched_sr_controller_.isGoalReached(&planner_util_, odom_helper_, cur_robot_pose_)) {
    ROS_INFO("Goal has been reached.");
    has_plan_ = false;
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
    ROS_ERROR("Failed to get current plan. Planning failure.");
    return false;
  } else if (local_plan.empty()) {
    ROS_ERROR("Plan is empty. Planning failure.");
    return false;
  } else {
    ROS_INFO("Sucessfully received a new plan.");
  }
  if (latched_sr_controller_.isPositionReached(&planner_util_, cur_robot_pose_)) {
    // reach here if the x,y pos of the goal has been reached, but we still need to rotate
    // some in order to orient with it
    bool latched_result = latched_sr_controller_.computeVelocityCommandsStopRotate(
        cmd_vel, planner_util_.getCurrentLimits().getAccLimits(),
        0.05,  // same val as in trajectory_generator_.setParameters call in BAUPlanner class
        &planner_util_, odom_helper_, cur_robot_pose_,
        boost::bind(&BAUPlanner::validate, bau_planner_, _1, _2, _3));  // yuck, boost
    if (latched_result) {
      ROS_INFO("Rotating towards goal");
    } else {
      ROS_INFO("Unable to rotate towards goal");
    }
    return latched_result;
  } else {
    geometry_msgs::PoseStamped drive_cmds;
    drive_cmds.header.frame_id = costmap_->getBaseFrameID();
    geometry_msgs::PoseStamped robot_vel;
    odom_helper_.getRobotVel(robot_vel);
    // try to find a new trajectory using the planner

    base_local_planner::Trajectory new_trajectory =
        bau_planner_->plan(cur_robot_pose_, robot_vel, costmap_->getRobotFootprint(), local_plan, drive_cmds);
    // pass the resulting trajectory on (may be 0, in the case of planning failure)
    cmd_vel.linear.x = drive_cmds.pose.position.x;
    cmd_vel.linear.y = drive_cmds.pose.position.y;
    cmd_vel.angular.z = tf2::getYaw(drive_cmds.pose.orientation);
    ROS_INFO("linear x vel %f", cmd_vel.linear.x);
    ROS_INFO("linear y vel %f", cmd_vel.linear.y);
    ROS_INFO("angular vel %f", cmd_vel.angular.z);
    if (new_trajectory.cost_ < 0) {
      ROS_ERROR("Unable to find a trajectory, stopping robot.");
      has_plan_ = false;
      return false;
    } else {
      has_plan_ = true;
      return true;
    }
  }
  return true;  // compiler satisfaction
}

bool BAUPlannerROS::isInitialized() { return initialised_; }

}  // namespace bau_local_planner
   // bau_local_planner
