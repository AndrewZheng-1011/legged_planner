#ifndef MULTI_COMMAND_INTERFACE_ROS_H
#define MULTI_COMMAND_INTERFACE_ROS_H

#include <ocs2_core/Types.h>
#include <ros/ros.h>

#include "legged_body_planner/robot_command/RobotCommand.h"

class MultiCommandInterfaceRos {
 public:
  MultiCommandInterfaceRos(ros::NodeHandle& nh, const std::string& topicPrefix);

  /**
   * @brief Adds a command interface | TODO: Check if interface already exists
   * perhaps..
   * @param robotCommand
   */
  void addCommandInterface(std::unique_ptr<RobotCommand> robotCommand);

  void spin();

 private:
  ros::NodeHandle nh_;

  /// Command interfaces
  std::unique_ptr<RobotCommand> robotCommandPtr_;
  std::vector<std::unique_ptr<RobotCommand>> robotCommands;

  double updateRate_;
};

#endif
