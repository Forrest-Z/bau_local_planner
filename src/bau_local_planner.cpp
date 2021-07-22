#include <bau_local_planner/bau_local_planner.h>

namespace bau_local_planner {

BAUPlanner::BAUPlanner(base_local_planner::LocalPlannerUtil *planner_util)
    : planner_util_(planner_util),
      obstacle_costs_(planner_util->getCostmap()),
      path_costs_(planner_util->getCostmap()),
      goal_costs_(planner_util->getCostmap(), 0.0, 0.0, true),        // no xshift or yshift, refer to the local goal
      goal_front_costs_(planner_util->getCostmap(), 0.0, 0.0, true),  // same as above
      alignment_costs_(planner_util->getCostmap()),
      path_scale_bias_(0.1f),
      goal_scale_bias_(0.1f),
      obstacle_scale_bias_(0.1f),
      twirling_scale_bias_(0.1f) {
  initialize();
}

bool BAUPlanner::validate(const Eigen::Vector3f position, const Eigen::Vector3f velocity,
                          const Eigen::Vector3f samples) {
  return true;
}

bool BAUPlanner::initialize() {
  using namespace base_local_planner;
  // first set up the cost functions that we'll use to evaluate generated trajectories
  setUpCostFunctions();
  // second set up the generators that will give us candidate trajectories to evaluate
  std::vector<TrajectorySampleGenerator *> trajectory_generators_;
  trajectory_generators_.push_back(&trajectory_generator_);
  // finally pass them all into the evaluator
  scored_sampling_planner_ = SimpleScoredSamplingPlanner(trajectory_generators_, cost_functions);
  return false;
}

void BAUPlanner::prePlan(const geometry_msgs::PoseStamped &robot_pose,
                         const std::vector<geometry_msgs::Point> &robot_footprint,
                         const std::vector<geometry_msgs::PoseStamped> &global_plan) {
  // first get rid of whatever plan we're currently following
  cur_plan_.clear();
  // our new plan is the global plan we've just received
  cur_plan_ = global_plan;

  // finally set up our cost functions with any new data that came in
  obstacle_costs_.setFootprint(robot_footprint);
  path_costs_.setTargetPoses(cur_plan_);
  goal_costs_.setTargetPoses(cur_plan_);
  alignment_costs_.setTargetPoses(cur_plan_);
}

void BAUPlanner::setUpCostFunctions() {
  // empty the current set of cost functions, in case we've toggled some at runtime
  cost_functions.clear();
  float path_dist_bias = planner_util_->getCostmap()->getResolution() * path_scale_bias_;
  float goal_dist_bias = planner_util_->getCostmap()->getResolution() * goal_scale_bias_;

  // set up the biases, i.e. how close we
  path_costs_.setScale(path_dist_bias);
  alignment_costs_.setScale(path_dist_bias);
  goal_costs_.setScale(goal_dist_bias);
  goal_front_costs_.setScale(goal_dist_bias);
  obstacle_costs_.setScale(obstacle_scale_bias_);
  twirling_costs_.setScale(twirling_scale_bias_);

  // set up the ones we have active for this planning tick
  oscillation_costs_.resetOscillationFlags();
  // TODO: add more config
  cost_functions.push_back(&oscillation_costs_);

  obstacle_costs_.setSumScores(true);
  cost_functions.push_back(&obstacle_costs_);
  cost_functions.push_back(&twirling_costs_);
  cost_functions.push_back(&path_costs_);
  cost_functions.push_back(&alignment_costs_);
}

base_local_planner::Trajectory BAUPlanner::plan(const geometry_msgs::PoseStamped &robot_pose,
                                                const geometry_msgs::PoseStamped &robot_vel,
                                                geometry_msgs::PoseStamped &drive_vel) {
  // begin the planning loop here -- first initialise the trajectory generator
  // requires everything in Eigen format for some reason ¯\_(ツ)_/¯
  base_local_planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits();

  trajectory_generator_.initialise(
      Eigen::Vector3f(robot_pose.pose.position.x, robot_pose.pose.position.y, tf2::getYaw(robot_pose.pose.orientation)),
      Eigen::Vector3f(robot_vel.pose.position.x, robot_vel.pose.position.y, tf2::getYaw(robot_vel.pose.orientation)),
      Eigen::Vector3f(cur_plan_.back().pose.position.x, cur_plan_.back().pose.position.y,
                      tf2::getYaw(cur_plan_.back().pose.orientation)),
      &limits, Eigen::Vector3f(5, 5, 5));

  return base_local_planner::Trajectory();
}

}  // namespace bau_local_planner
