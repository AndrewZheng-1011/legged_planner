#include <grid_map_core/iterators/iterators.hpp>

#include "legged_body_planner/MultiCommandInterfaceRos.h"
#include "legged_body_planner/robot_command/RobotGoalCommand.h"
#include "legged_body_planner/robot_command/RobotPlanCommand.h"
#include "legged_body_planner/robot_command/RobotVelocityCommand.h"

int main(int argc, char **argv) {
  // Initialize parameters
  std::cerr << "[Multi-Command Interface ROS] Initialized node" << std::endl;
  ros::init(argc, argv, "multi_command_interface_ros_node");
  ros::NodeHandle nh;
  const std::string robotName = "legged_robot";
  planning_utils::PlannerConfig plannerConfig;
  plannerConfig.loadParams(nh);

  // Initialize multi command interface
  MultiCommandInterfaceRos multiCommandInterfaceRos{nh, robotName};

  multiCommandInterfaceRos.addCommandInterface(
      std::make_unique<RobotVelocityCommand>(nh, "/cmd_vel", robotName,
                                             plannerConfig));
  multiCommandInterfaceRos.addCommandInterface(
      std::make_unique<RobotGoalCommand>(nh, "/move_base_simple/goal",
                                         robotName, plannerConfig));
  multiCommandInterfaceRos.addCommandInterface(
      std::make_unique<RobotPlanCommand>(nh, "/plan", robotName,
                                         plannerConfig));

  // Start interface | TODO (AZ): Make the callback asynchronous s.t. each
  // command process callbacks at diff rate
  multiCommandInterfaceRos.spin();
  return 0;
}
