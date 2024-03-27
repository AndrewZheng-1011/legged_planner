#include "legged_body_planner/MultiCommandInterfaceRos.h"

#include <legged_body_utils/planning_utils.h>

#include "legged_body_planner/robot_command/RobotVelocityCommand.h"

MultiCommandInterfaceRos::MultiCommandInterfaceRos(
    ros::NodeHandle& nh, const std::string& topicPrefix) {
  nh_ = nh;

  // Load params
  planning_utils::loadROSParam(nh_, "/multi_command_interface_ros/update_rate",
                               updateRate_);
}

void MultiCommandInterfaceRos::addCommandInterface(
    std::unique_ptr<RobotCommand> robotCommand) {
  robotCommands.emplace_back(std::move(robotCommand));
}

void MultiCommandInterfaceRos::spin() {
  ros::Rate r(updateRate_);

  while (ros::ok()) {
    // Process callbacks from command interface
    for (auto it = robotCommands.begin(); it != robotCommands.end(); ++it) {
      it->get()->spinOnce();
    }
    // robotCommandPtr_->spinOnce();

    r.sleep();  // TODO (AZ): Use ros::Time sleepUntil() or stl lib
  }
}
