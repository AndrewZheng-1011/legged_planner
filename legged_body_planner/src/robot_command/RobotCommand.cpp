#include "legged_body_planner/robot_command/RobotCommand.h"

RobotCommand::RobotCommand(ros::NodeHandle& nh,
                           const std::string& robotCommandTopic,
                           const std::string& topicPrefix) {
  // Set up observation subscriber
  auto observationCallback =
      [this](const ocs2_msgs::mpc_observation::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(latestObservationMutex_);
        latestObservation_ =
            ocs2::ros_msg_conversions::readObservationMsg(*msg);
      };
  observationSub_ = nh.subscribe<ocs2_msgs::mpc_observation>(
      topicPrefix + "_mpc_observation", 1, observationCallback);

  // Set up publisher | TODO (AZ): It is not working, do some print statements
  targetTrajectoriesPublisherPtr_.reset(
      new ocs2::TargetTrajectoriesRosPublisher(nh, topicPrefix));
}

void RobotCommand::spinOnce() { ros::spinOnce(); }