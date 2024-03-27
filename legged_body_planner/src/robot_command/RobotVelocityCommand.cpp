#include "legged_body_planner/robot_command/RobotVelocityCommand.h"

#include <geometry_msgs/Twist.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

RobotVelocityCommand::RobotVelocityCommand(
    ros::NodeHandle& nh, const std::string& robotCommandTopic,
    const std::string& topicPrefix, planning_utils::PlannerConfig plannerConfig)
    : RobotCommand{nh, robotCommandTopic, topicPrefix},
      plannerConfig_{plannerConfig} {
  // TODO (AZ): Might need to create copy of nodehandle since nh goes out of
  // scope

  // Load params

  // Create subscriber callbacks
  auto cmdVelCallback = [this](const geometry_msgs::Twist::ConstPtr& msg) {
    if (this->latestObservation_.time == 0.0) {
      return;
    }

    ocs2::vector_t cmdVel{4};
    cmdVel << msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.z;

    this->targetTrajectories_ =
        this->robotInputToTargetTrajectories(cmdVel, this->latestObservation_);
    this->targetTrajectoriesPublisherPtr_->publishTargetTrajectories(
        this->targetTrajectories_);
  };

  sub_ =
      nh.subscribe<geometry_msgs::Twist>(robotCommandTopic, 1, cmdVelCallback);
}

ocs2::TargetTrajectories RobotVelocityCommand::robotInputToTargetTrajectories(
    const ocs2::vector_t& input, const ocs2::SystemObservation& observation) {
  // Get current state
  const ocs2::vector_t currentPose = observation.state.segment<6>(6);

  // Get rotate cmd_vel
  const Eigen::Matrix<ocs2::scalar_t, 3, 1> ypr =
      currentPose.tail(3);  // YPR of euler angles
  ocs2::vector_t cmdVelRot = ocs2::getRotationMatrixFromZyxEulerAngles(ypr) *
                             input.head(3);  // cmd vel w.r.t. body frame
  const ocs2::scalar_t timeToTarget = plannerConfig_.PLAN_HORIZON;

  // Define target pose
  ocs2::vector_t target(6);
  target(0) = currentPose(0) + cmdVelRot(0) * timeToTarget;
  target(1) = currentPose(1) + cmdVelRot(1) * timeToTarget;
  target(2) = plannerConfig_.COM_HEIGHT;
  target(3) = currentPose(3) + input(3) * timeToTarget;  // yaw
  target(4) = 0;
  target(5) = 0;

  // Target reaching time
  const ocs2::scalar_t targetReachingTime =
      observation.time + timeToTarget;  // Fixed time

  auto trajectories =
      targetPoseToTargetTrajectories(target, observation, targetReachingTime);

  // Set initial and final velocity to be the following
  trajectories.stateTrajectory[0].head(3) = cmdVelRot;
  trajectories.stateTrajectory[1].head(3) = cmdVelRot;

  return trajectories;
}

ocs2::TargetTrajectories RobotVelocityCommand::targetPoseToTargetTrajectories(
    const ocs2::vector_t& targetPose,
    const ocs2::SystemObservation& observation,
    const ocs2::scalar_t targetReachingTime) {
  // Desired time trajectory
  const ocs2::scalar_array_t timeTrajectory{
      observation.time, targetReachingTime};  // Initiaize & declare
                                              // std::vector @ t0 and t_target
  // Desired state trajecotry
  ocs2::vector_t currentPose =
      observation.state.segment<6>(6);  // x, y, z, y, p, r
  currentPose(2) = plannerConfig_.COM_HEIGHT;
  currentPose(4) = 0;  // pitch
  currentPose(5) = 0;  // roll

  // Create std::vector to contain initial and final state trajectory
  ocs2::vector_array_t stateTrajectory(
      2, ocs2::vector_t::Zero(observation.state.size()));

  // State trajectory: x_dot, euler_rate, x, euler, and q's
  stateTrajectory[0] << ocs2::vector_t::Zero(6), currentPose,
      plannerConfig_.DEFAULT_JOINT_STATES;
  stateTrajectory[1] << ocs2::vector_t::Zero(6), targetPose,
      plannerConfig_.DEFAULT_JOINT_STATES;

  // Desired input traj (just get right dim, not used here)
  const ocs2::vector_array_t inputTrajectory(
      2, ocs2::vector_t::Zero(observation.input.size()));
  return ocs2::TargetTrajectories(timeTrajectory, stateTrajectory,
                                  inputTrajectory);
}

void RobotVelocityCommand::spinOnce() { ros::spinOnce(); }