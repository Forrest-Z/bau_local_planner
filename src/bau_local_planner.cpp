#include <bau_local_planner.h>

namespace bau_local_planner {

BAUPlanner() : planner_util_(planner_util) {}

bool BAUPlanner::validate(const Eigen::Vector3f position, const Eigen::Vector3f velocity,
                          const Eigen::Vector3f samples) {
  return true;
}

base_local_planner::Trajectory plan(const geometry_msgs::PoseStamped &global_pose,
                                    const geometry_msgs::PoseStamped &global_vel,
                                    geometry_msgs::PoseStamped &drive_velocities) {}

}  // namespace bau_local_planner
