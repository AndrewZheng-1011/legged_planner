#include "legged_body_planner/robot_command/RobotPlanCommand.h"

#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/iterators.hpp>

RobotPlanCommand::RobotPlanCommand(ros::NodeHandle& nh,
                                   const std::string& robotCommandTopic,
                                   const std::string& topicPrefix,
                                   planning_utils::PlannerConfig& plannerConfig)
    : RobotCommand{nh, robotCommandTopic, topicPrefix},
      leggedRobotAdapter_{plannerConfig} {
  // Load params
  std::string planTerminateTopic;

  // planning_utils::loadROSParam(nh, "topics/plan",
  //                              planTerminateTopic);  // TODO (AZ): Use this
  // planning_utils::loadROSParam(nh, "/legged_body_planner/replan", replan_);

  // targetTrajectories_.clear();

  // Subscriber
  auto planCallback = [this](const legged_body_msgs::Plan::ConstPtr& plan) {
    if (this->latestObservation_.time == 0.0) {
      return;
    }
    // Translate generic plan to legged plan

    // Convert to target trajectories
    this->targetTrajectories_ =
        this->planToTargetTrajectories(plan, this->latestObservation_);
    this->targetTrajectoriesPublisherPtr_->publishTargetTrajectories(
        this->targetTrajectories_);
  };
  sub_ =
      nh.subscribe<legged_body_msgs::Plan>(robotCommandTopic, 1, planCallback);
}

ocs2::TargetTrajectories RobotPlanCommand::planToTargetTrajectories(
    const legged_body_msgs::Plan::ConstPtr& plan,
    const ocs2::SystemObservation& observation) {
  /// Convert plan to eigen datatypes

  // Time
  ocs2::scalar_array_t timeTrajectories(plan->times.begin(), plan->times.end());

  // States
  std::size_t N = plan->states.size();  // N : # of trajectory
  timeTrajectories.resize(N);

  ocs2::vector_array_t stateTrajectories(N);
  for (std::size_t i = 0; i < N; i++) {
    const std::vector<ocs2::scalar_t> stateVal(
        plan->states[i].value.begin(),
        plan->states[i].value.end());  // Convert to double
    planning_utils::stdVecToEigen(stateVal, stateTrajectories[i]);
  }

  // Control
  N = plan->controls.size();
  ocs2::vector_array_t controlTrajectories(N);
  for (std::size_t i = 0; i < N; i++) {
    const std::vector<ocs2::scalar_t> ctrl_val(
        plan->controls[i].value.begin(),
        plan->controls[i].value.end());  // Convert to double
    planning_utils::stdVecToEigen(ctrl_val, controlTrajectories[i]);
  }

  // Convert to legged states
  leggedRobotAdapter_.adaptMotion(timeTrajectories, stateTrajectories,
                                  controlTrajectories);

  return ocs2::TargetTrajectories(timeTrajectories, stateTrajectories,
                                  controlTrajectories);
}

ocs2::TargetTrajectories RobotPlanCommand::robotInputToTargetTrajectories(
    const ocs2::vector_t& input, const ocs2::SystemObservation& observation) {
  // TODO (AZ): Somehow fix architecture s.t. can utilize this and not keep it
  // empty and not used
  if (!targetTrajectories_.empty()) {
    ocs2::vector_array_t inputTrajectory(
        2, ocs2::vector_t::Zero(observation.input.size()));
    ocs2::vector_array_t stateTrajectory(
        2, ocs2::vector_t::Zero(observation.state.size()));
    ocs2::scalar_array_t timeTrajectory{observation.time,
                                        observation.time + 1.0};
    return ocs2::TargetTrajectories(timeTrajectory, stateTrajectory,
                                    inputTrajectory);
  }
  return targetTrajectories_;
}

void RobotPlanCommand::spinOnce() { ros::spinOnce(); }