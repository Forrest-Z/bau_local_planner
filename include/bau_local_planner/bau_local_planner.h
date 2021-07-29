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
#include <tf2/utils.h>

namespace bau_local_planner {
/**
 * @brief The BAUPlanner class is the core local planner, it implements a basic approach of
 *        generating a series of candidate trajectories and then evaluating them according to
 *        a set of pre-defined cost functions to find ones that will best produce our desired behaviour.
 */
class BAUPlanner {
 public:
  /**
   * @brief Constructor for the ROS wrapper for the BAU local planner.
   * @param planner_util - the local planning helper class, instantiated by BAUPlannerROS
   */
  BAUPlanner(base_local_planner::LocalPlannerUtil *planner_util);

  /**
   * @brief Initialisation procedure for the local planner, sets up trajectory,
   *        generators and scoring system.
   * @returns true if the initialisation procedure was successful, false if not
   */
  bool initialize();

  /**
   * @brief Used only as a lambda by systems outside this class that may wish to make
   *        use of the output from one or more cost functions to evaluate trajectories
   *        they have generated independently. For now this is used in our call to
   *        LatchedStopRotateController::computeVelocityCommandsStopRotate
   *        which calls this function in LatchedStopRotateController::stopWithAccLimits
   * @param pos - robot pose
   * @param vel - robot velocity
   * @param num_samples - num samples in each dimension to take in Eigen format
   * @returns true if the proposed trajectory is valid with respect to our cost functions
   *          false if not
   */
  bool validate(Eigen::Vector3f pos, Eigen::Vector3f vel, Eigen::Vector3f vel_samples);

  /**
   * @brief Sets up the group of cost functions that will be used to evaluate
   *        generated trajectories.
   */
  void setUpCostFunctions();

  /**
   * @brief Performs the actual local planning. Generates a set of local trajectories
   *        evaluates them according to the defined cost functions, and if a valid
   *        trajectory is found, sends back the velocity commands in order to execute it.
   * @param robot_pose - the current pose of the robot
   * @param robot_vel - the current velocity of the robot
   * @param robot_footprint - the robot footprint (from costmap)
   * @param local_plan - the global plan translated to local frame
   * @param cmd_vel - the drive commands to be sent to the controller if a valid trajectory is found
   * @returns the best trajectory found after the planning procedure is complete
   */
  base_local_planner::Trajectory plan(const geometry_msgs::PoseStamped &robot_pose,
                                      const geometry_msgs::PoseStamped &robot_vel,
                                      const std::vector<geometry_msgs::Point> &robot_footprint,
                                      const std::vector<geometry_msgs::PoseStamped> &local_plan,
                                      geometry_msgs::PoseStamped &cmd_vel);

 private:
  base_local_planner::LocalPlannerUtil *planner_util_;  //!< helper class for local planning
  base_local_planner::SimpleScoredSamplingPlanner
      scored_sampling_planner_;                                         //!< used for evaluating generated trajectories
  base_local_planner::SimpleTrajectoryGenerator trajectory_generator_;  //!< used for generating candidate trajectories
  // cost functions for local planning
  std::vector<base_local_planner::TrajectoryCostFunction *>
      cost_functions_;  //!< container of cost functions used to evaluate generated trajectories
  base_local_planner::OscillationCostFunction
      oscillation_costs_;  //!< penalises trajectories that cause rotational oscillations
  base_local_planner::ObstacleCostFunction
      obstacle_costs_;  //!< penalises trajectories where the robot footprint would cross into an obstacle
  base_local_planner::MapGridCostFunction path_costs_;  //!< penalises trajectories that stray far from the global path
  base_local_planner::MapGridCostFunction goal_costs_;  //!< penalises trajectories that stray far from the goal
  base_local_planner::MapGridCostFunction
      goal_facing_costs_;  //!< penalises trajectories that stray far from the global path
  base_local_planner::MapGridCostFunction alignment_costs_;  //!< penalises trajectories not aligned with global path
  const float path_scale_bias_;                              //!< relative importance of the path cost function
  const float goal_scale_bias_;                              //!< relative importance of the goal cost function
  const float obstacle_scale_bias_;                          //!< relative importance of the obstacle cost function
  const float x_shift_distance_;                      //!< distance, in metres, to look ahead of current robot position
  const int num_sampled_trajectories_;                //!< num of candidate trajectories to generate in each dimension
  std::vector<geometry_msgs::PoseStamped> cur_plan_;  //!< the current global plan translated to a local frame
};

}  // namespace bau_local_planner
