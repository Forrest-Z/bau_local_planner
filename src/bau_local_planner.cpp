#include <bau_local_planner/bau_local_planner.h>

namespace bau_local_planner {

BAUPlanner::BAUPlanner(base_local_planner::LocalPlannerUtil *planner_util)
    : planner_util_(planner_util),
      obstacle_costs_(planner_util->getCostmap()),
      path_costs_(planner_util->getCostmap()),
      goal_costs_(planner_util->getCostmap(), 0.0, 0.0, true),         // no xshift or yshift, refer to the local goal
      goal_facing_costs_(planner_util->getCostmap(), 0.0, 0.0, true),  // same as above
      alignment_costs_(planner_util->getCostmap()),
      path_scale_bias_(32.0),
      goal_scale_bias_(20.0),
      obstacle_scale_bias_(0.01f),
      x_shift_distance_(0.2),  // metres
      num_sampled_trajectories_(20) {
  initialize();
}

bool BAUPlanner::initialize() {
  using namespace base_local_planner;
  // first set up the cost functions that we'll use to evaluate generated trajectories
  setUpCostFunctions();
  // second set up the generators that will give us candidate trajectories to evaluate
  trajectory_generator_.setParameters(1.7f, 0.025f, 0.1f, true,
                                      0.05f);  // performance is very sensitive to these params
  std::vector<TrajectorySampleGenerator *> trajectory_generators_;
  // TODO: We can add more here if needed
  trajectory_generators_.push_back(&trajectory_generator_);
  // finally pass them all into the evaluator
  scored_sampling_planner_ = SimpleScoredSamplingPlanner(trajectory_generators_, cost_functions_);
  return true;
}

bool BAUPlanner::validate(Eigen::Vector3f pos, Eigen::Vector3f vel, Eigen::Vector3f vel_samples) {
  // first grab the goal pose, which is the pose at the back of the current plan
  Eigen::Vector3f goal(cur_plan_.back().pose.position.x, cur_plan_.back().pose.position.y,
                       tf2::getYaw(cur_plan_.back().pose.orientation));
  // we only wound up using one trajectory generator, so just pull it out here and initialise it
  // with all the above information
  base_local_planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits();
  trajectory_generator_.initialise(
      pos, vel, goal, &limits,
      Eigen::Vector3f(num_sampled_trajectories_, num_sampled_trajectories_, num_sampled_trajectories_));
  // we generate a trajectory to reach this goal pose with respect to our cost functions
  base_local_planner::Trajectory result;
  trajectory_generator_.generateTrajectory(pos, vel, vel_samples, result);

  // here we score this trajectory according to our cost functions
  // returning a cost of -1 if it is invalid
  if (scored_sampling_planner_.scoreTrajectory(result, -1) < 0) {
    return false;
  }
  return true;
}

void BAUPlanner::setUpCostFunctions() {
  // empty the current set of cost functions, in case we've toggled some at runtime
  cost_functions_.clear();
  float path_dist_bias = planner_util_->getCostmap()->getResolution() * path_scale_bias_;
  float goal_dist_bias = planner_util_->getCostmap()->getResolution() * goal_scale_bias_;
  ROS_INFO("path scale: %f", path_dist_bias);
  ROS_INFO("goal scale: %f", goal_dist_bias);

  // set up the biases, i.e. how close we
  path_costs_.setScale(path_dist_bias);
  alignment_costs_.setScale(path_dist_bias);
  goal_costs_.setScale(goal_dist_bias);
  goal_facing_costs_.setScale(goal_dist_bias);
  obstacle_costs_.setScale(obstacle_scale_bias_);
  obstacle_costs_.setParams(0.22, 0.2, 0.25);

  goal_facing_costs_.setStopOnFailure(false);
  alignment_costs_.setStopOnFailure(false);

  // sets a point x_shift_distance_ metres in front of the robot
  goal_facing_costs_.setXShift(x_shift_distance_);
  alignment_costs_.setXShift(x_shift_distance_);

  oscillation_costs_.resetOscillationFlags();
  obstacle_costs_.setSumScores(true);
  oscillation_costs_.setOscillationResetDist(0.05, 0.2);

  // set up the ones we have active for this planning tick
  // TODO: could add the option to enable/disable certain cost functions at runtime here
  cost_functions_.push_back(&oscillation_costs_);
  cost_functions_.push_back(&obstacle_costs_);
  cost_functions_.push_back(&path_costs_);
  cost_functions_.push_back(&goal_costs_);
  cost_functions_.push_back(&goal_facing_costs_);
  cost_functions_.push_back(&alignment_costs_);
}

base_local_planner::Trajectory BAUPlanner::plan(const geometry_msgs::PoseStamped &robot_pose,
                                                const geometry_msgs::PoseStamped &robot_vel,
                                                const std::vector<geometry_msgs::Point> &robot_footprint,
                                                const std::vector<geometry_msgs::PoseStamped> &local_plan,
                                                geometry_msgs::PoseStamped &cmd_vel) {
  // first get rid of whatever plan we're currently following
  cur_plan_.clear();
  // our new plan is the plan we've just received
  cur_plan_ = local_plan;

  // set up our cost functions with any new data that came in
  obstacle_costs_.setFootprint(robot_footprint);
  path_costs_.setTargetPoses(cur_plan_);
  goal_costs_.setTargetPoses(cur_plan_);
  alignment_costs_.setTargetPoses(cur_plan_);
  goal_facing_costs_.setTargetPoses(cur_plan_);

  // begin planning loop here -- first initialise the trajectory generator
  // requires everything in Eigen format for some reason ¯\_(ツ)_/¯
  base_local_planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits();

  trajectory_generator_.initialise(
      Eigen::Vector3f(robot_pose.pose.position.x, robot_pose.pose.position.y, tf2::getYaw(robot_pose.pose.orientation)),
      Eigen::Vector3f(robot_vel.pose.position.x, robot_vel.pose.position.y, tf2::getYaw(robot_vel.pose.orientation)),
      Eigen::Vector3f(cur_plan_.back().pose.position.x, cur_plan_.back().pose.position.y,
                      tf2::getYaw(cur_plan_.back().pose.orientation)),
      &limits,
      Eigen::Vector3f(num_sampled_trajectories_, num_sampled_trajectories_,
                      num_sampled_trajectories_));  // TODO: Make sampling sizes editable
  // stores the best-scoring output of trajectory search
  // first initialise a blank trajectory
  base_local_planner::Trajectory result;
  // initialise the cost to negative, so we know if the planner fails to find a new path (i.e. this value has not been
  // overwritten)
  result.cost_ = -1;
  // run trajectory search,
  // TODO: could pass a container rather than NULL to retreive all candidate trajectories for visualisation, for now
  // just discard them
  std::vector<base_local_planner::Trajectory> all_explored;
  scored_sampling_planner_.findBestTrajectory(result, &all_explored);
  ROS_INFO("explored %d trajectories", (int)all_explored.size());
  ROS_INFO("final result cost: %f", result.cost_);

  // if the cost is < 0 (set above to -1) the generator didn't find a usable trajectory
  // so the best thing to do is stop the robot, so report back 0 velocities in all dimensions
  if (result.cost_ < 0) {
    cmd_vel.pose.position.x = 0;
    cmd_vel.pose.position.y = 0;
    cmd_vel.pose.position.z = 0;
    cmd_vel.pose.orientation.w = 1;
    cmd_vel.pose.orientation.x = 0;
    cmd_vel.pose.orientation.y = 0;
    cmd_vel.pose.orientation.z = 0;
  } else {
    // else, we did find a usable trajectory, so we pass this up so it can be
    // sent on to the controller
    ROS_INFO("final result xv: %f", result.xv_);
    ROS_INFO("final result yv: %f", result.yv_);
    ROS_INFO("final result rv: %f", result.thetav_);
    cmd_vel.pose.position.x = result.xv_;
    cmd_vel.pose.position.y = result.yv_;
    cmd_vel.pose.position.z = 0.0;  // not used, but just so every value is initialised
    tf2::Quaternion q;
    q.setRPY(0, 0, result.thetav_);
    tf2::convert(q, cmd_vel.pose.orientation);
  }

  // update this cost function with information about the new trajectory, useful to give
  // a smoother transition on the next iteration
  oscillation_costs_.updateOscillationFlags(
      Eigen::Vector3f(robot_pose.pose.position.x, robot_pose.pose.position.y, tf2::getYaw(robot_pose.pose.orientation)),
      &result, planner_util_->getCurrentLimits().min_vel_trans);

  return result;
}

}  // namespace bau_local_planner
