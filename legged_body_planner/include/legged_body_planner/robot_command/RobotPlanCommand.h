#ifndef ROBOT_PLAN_COMMAND_H
#define ROBOT_PLAN_COMMAND_H

#include <legged_body_utils/planning_utils.h>

#include "legged_body_msgs/Plan.h"
#include "legged_body_planner/motion_adapters/LeggedRobotAdapter.h"
#include "legged_body_planner/robot_command/RobotCommand.h"

/**
 * @brief RobotPlanCommand is a ros-based class that is a middle-layer that
 * resides between the planner and trajectories to be utilized in lower-level
 * motion planning/control.
 *
 * This class ensures that the plan given by user is translated to a plan
 * appropriate for legged systems.
 *
 * TODO (AZ): Make this inheritance structure from RobotCommand more clean s.t.
 * use one virtual method to implement for various data types and not just
 * vector_t
 *
 */
class RobotPlanCommand : public RobotCommand {
 public:
  RobotPlanCommand(ros::NodeHandle& nh, const std::string& robotCommandTopic,
                   const std::string& topicPrefix,
                   planning_utils::PlannerConfig& plannerConfig);
  void spinOnce() override;

 protected:
  ocs2::TargetTrajectories robotInputToTargetTrajectories(
      const ocs2::vector_t& input,
      const ocs2::SystemObservation& observation) override;

  /**
   * @brief Takes a plan and convert to target trajectory
   * @param inputs
   * @param observation
   * @return ocs2::TargetTrajectories
   */
  virtual ocs2::TargetTrajectories planToTargetTrajectories(
      const legged_body_msgs::Plan::ConstPtr& plan,
      const ocs2::SystemObservation& observation);

  LeggedRobotAdapter leggedRobotAdapter_;
};

#endif  // ROBOT_PLAN_COMMAND_H
