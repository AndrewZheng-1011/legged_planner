#ifndef ROBOT_GOAL_COMMAND_H
#define ROBOT_GOAL_COMMAND_H
#include <legged_body_utils/planning_utils.h>
#include <tf2_ros/transform_listener.h>

#include "legged_body_planner/robot_command/RobotCommand.h"

class RobotGoalCommand : public RobotCommand {
 public:
  RobotGoalCommand(ros::NodeHandle& nh, const std::string& robotCommandTopic,
                   const std::string& topicPrefix,
                   planning_utils::PlannerConfig& plannerConfig);

  void spinOnce() override;

 protected:
  ocs2::TargetTrajectories robotInputToTargetTrajectories(
      const ocs2::vector_t& input,
      const ocs2::SystemObservation& observation) override;

  virtual ocs2::TargetTrajectories targetPoseToTargetTrajectories(
      const ocs2::vector_t& targetPose,
      const ocs2::SystemObservation& observation,
      const ocs2::scalar_t targetReachingTime);

  planning_utils::PlannerConfig plannerConfig_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf2_;
};

#endif  // ROBOT_GOAL_COMMAND_H
