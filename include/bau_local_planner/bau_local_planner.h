#pragma once
#include <base_local_planner/local_planner_util.h>
#include <base_local_planner/simple_scored_sampling_planner.h>
#include <base_local_planner/simple_trajectory_generator.h>
#include <nav_core/base_local_planner.h>

namespace bau_local_planner {

class BAUPlanner {
 public:
  BAUPlanner(std::shared_ptr<base_local_planner::LocalPlannerUtil> planner_util);

  bool validate(const Eigen::Vector3f position, const Eigen::Vector3f velocity, const Eigen::Vector3f samples);

  base_local_planner::Trajectory plan(const geometry_msgs::PoseStamped &global_pose,
                                      const geometry_msgs::PoseStamped &global_vel,
                                      geometry_msgs::PoseStamped &drive_velocities);

 private:
  std::shared_ptr<base_local_planner::LocalPlannerUtil> planner_util_;
  base_local_planner::SimpleScoredSamplingPlanner scored_sampling_planner_;
}

}  // namespace bau_local_planner
