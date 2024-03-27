#ifndef ROBOT_VELOCITY_COMMAND_H
#define ROBOT_VELOCITY_COMMAND_H

#include <legged_body_utils/planning_utils.h>

#include "legged_body_planner/robot_command/RobotCommand.h"

class RobotVelocityCommand : public RobotCommand {
 public:
  RobotVelocityCommand(ros::NodeHandle& nh,
                       const std::string& robotCommandTopic,
                       const std::string& topicPrefix,
                       planning_utils::PlannerConfig plannerConfig);

  void spinOnce() override;

 protected:
  ocs2::TargetTrajectories robotInputToTargetTrajectories(
      const ocs2::vector_t& input,
      const ocs2::SystemObservation& observation) override;
  //   ocs2::TargetTrajectories robotInputToTargetTrajectories(
  //       const ocs2::vector_t& input,
  //       const ocs2::SystemObservation& observation) override;

  virtual ocs2::TargetTrajectories targetPoseToTargetTrajectories(
      const ocs2::vector_t& targetPose,
      const ocs2::SystemObservation& observation,
      const ocs2::scalar_t targetReachingTime);

  planning_utils::PlannerConfig plannerConfig_;
};

#endif  // ROBOT_VELOCITY_COMMAND_H
