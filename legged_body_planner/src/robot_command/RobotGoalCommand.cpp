#include "legged_body_planner/robot_command/RobotGoalCommand.h"

#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

RobotGoalCommand::RobotGoalCommand(ros::NodeHandle& nh,
                                   const std::string& robotCommandTopic,
                                   const std::string& topicPrefix,
                                   planning_utils::PlannerConfig& plannerConfig)
    : RobotCommand{nh, robotCommandTopic, topicPrefix},
      tf2_{buffer_},
      plannerConfig_{plannerConfig} {
  // Create subscriber callback
  auto goalCallback = [this](const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (this->latestObservation_.time == 0) {
      return;
    }

    geometry_msgs::PoseStamped pose = *msg;
    try {
      ros::Duration(0.2);
      this->buffer_.transform(
          pose, pose, "odom",
          ros::Duration(0.2));  // Simply putting pose msg into odom?
    } catch (tf2::TransformException& ex) {
      ROS_WARN("Failure %s\n", ex.what());
      return;
    }
    /// Convert msg to goal command
    // Convert to quaternion
    Eigen::Quaternion<ocs2::scalar_t> q{
        pose.pose.orientation.w, pose.pose.orientation.x,
        pose.pose.orientation.y, pose.pose.orientation.z};

    ocs2::vector_t cmdGoal{6};
    cmdGoal << pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
        q.toRotationMatrix().eulerAngles(0, 1, 2).z(),
        q.toRotationMatrix().eulerAngles(0, 1, 2).y(),
        q.toRotationMatrix().eulerAngles(0, 1, 2).x();

    this->targetTrajectories_ =
        this->robotInputToTargetTrajectories(cmdGoal, this->latestObservation_);
    this->targetTrajectoriesPublisherPtr_->publishTargetTrajectories(
        this->targetTrajectories_);
  };

  sub_ = nh.subscribe<geometry_msgs::PoseStamped>(robotCommandTopic, 1,
                                                  goalCallback);
}

ocs2::TargetTrajectories RobotGoalCommand::robotInputToTargetTrajectories(
    const ocs2::vector_t& input, const ocs2::SystemObservation& observation) {
  // Current position (x, ypr) in R^6
  const ocs2::vector_t currentPose = observation.state.segment<6>(6);
  ocs2::vector_t targetPose{6};  // x,y,z,y,p,r
  targetPose << input[0], input[1], plannerConfig_.COM_HEIGHT, input[3], 0.0,
      0.0;

  auto estimateTimeToTarget = [this](const ocs2::vector_t& pos1,
                                     const ocs2::vector_t& pos2) {
    const ocs2::vector_t differencePos = pos2 - pos1;
    const ocs2::scalar_t& dx = differencePos[0];
    const ocs2::scalar_t& dy = differencePos[1];
    const ocs2::scalar_t& dyaw = differencePos[2];
    const ocs2::scalar_t& rotationTime =
        std::abs(dyaw) / this->plannerConfig_.TARGET_ROTATION_VELOCITY;
    const ocs2::scalar_t& displacement = std::sqrt(dx * dx + dy * dy);
    const ocs2::scalar_t& displacement_time =
        displacement / this->plannerConfig_.TARGET_DISPLACEMENT_VELOCITY;
    return std::max(rotationTime, displacement_time);
  };

  const ocs2::scalar_t targetReachingTime{
      observation.time + estimateTimeToTarget(currentPose, targetPose)};

  return targetPoseToTargetTrajectories(targetPose, observation,
                                        targetReachingTime);
}

ocs2::TargetTrajectories RobotGoalCommand::targetPoseToTargetTrajectories(
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
      2, ocs2::vector_t::Zero(
             observation.state.size()));  // TODO: Deal w/ velocity heuristic

  // State trajectory: x_dot, euler_rate, x, euler, and q's | TODO (AZ): state
  // trajectory with spline
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

void RobotGoalCommand::spinOnce() { ros::spinOnce(); }