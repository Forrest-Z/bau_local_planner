#pragma once
#include <base_local_planner/local_planner_util.h>
#include <nav_core/base_local_planner.h>
// trajectory generators/scoring tools
#include <base_local_planner/simple_scored_sampling_planner.h>
#include <base_local_planner/simple_trajectory_generator.h>
// cost functions for evaluating generated trajectoreis
#include <base_local_planner/map_grid_cost_function.h>
#include <base_local_planner/obstacle_cost_function.h>
#include <base_local_planner/oscillation_cost_function.h>
#include <base_local_planner/twirling_cost_function.h>
#include <tf2/utils.h>

namespace bau_local_planner {

class BAUPlanner {
 public:
  BAUPlanner(base_local_planner::LocalPlannerUtil *planner_util);

  bool validate(const Eigen::Vector3f position, const Eigen::Vector3f velocity, const Eigen::Vector3f samples);

  bool initialize();
  void setUpCostFunctions();

  void prePlan(const geometry_msgs::PoseStamped &robot_pose, const std::vector<geometry_msgs::Point> &robot_footprint,
               const std::vector<geometry_msgs::PoseStamped> &global_plan);

  base_local_planner::Trajectory plan(const geometry_msgs::PoseStamped &robot_pose,
                                      const geometry_msgs::PoseStamped &robot_vel,
                                      geometry_msgs::PoseStamped &drive_vel);

 private:
  base_local_planner::LocalPlannerUtil *planner_util_;
  base_local_planner::SimpleScoredSamplingPlanner scored_sampling_planner_;
  base_local_planner::SimpleTrajectoryGenerator trajectory_generator_;
  // cost functions for local planning
  std::vector<base_local_planner::TrajectoryCostFunction *> cost_functions;
  base_local_planner::OscillationCostFunction oscillation_costs_;
  base_local_planner::ObstacleCostFunction obstacle_costs_;
  base_local_planner::MapGridCostFunction path_costs_;
  base_local_planner::MapGridCostFunction goal_costs_;
  base_local_planner::MapGridCostFunction goal_front_costs_;
  base_local_planner::MapGridCostFunction alignment_costs_;
  base_local_planner::TwirlingCostFunction twirling_costs_;
  float path_scale_bias_;
  float goal_scale_bias_;
  float obstacle_scale_bias_;
  float twirling_scale_bias_;
  std::vector<geometry_msgs::PoseStamped> cur_plan_;
};

}  // namespace bau_local_planner
