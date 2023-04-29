#include "legged_body_planner/legged_body_planner.h"

int main(int argc, char **argv) {
  std::cout << "Load legged body planner node" << std::endl;
  ros::init(argc, argv, "legged_body_planner_node");
  ros::NodeHandle nh;
  std::string robot_name = "legged_robot";

  LeggedBodyPlanner legged_body_planner(nh, robot_name);
  legged_body_planner.spin();

  return 0;
}