#include <gtest/gtest.h>
#include <ros/ros.h>

#include "legged_body_planner/robot_command/RobotCommand.h"
#include "legged_body_planner/robot_command/RobotGoalCommand.h"
#include "legged_body_planner/robot_command/RobotPlanCommand.h"
#include "legged_body_planner/robot_command/RobotVelocityCommand.h"

class RobotCommandTestFixture : public ::testing::Test {
 public:
  RobotCommandTestFixture() : robotName{"legged_robot"} {
    plannerConfig_.loadParams(nh);
  }
  ros::NodeHandle nh;
  const std::string robotName;
  planning_utils::PlannerConfig plannerConfig_;
};

TEST_F(RobotCommandTestFixture, RobotVelocityCommandInit) {
  const std::string topic = "/cmd_vel";
  std::unique_ptr<RobotCommand> robotCommand =
      std::make_unique<RobotVelocityCommand>(nh, topic, robotName,
                                             plannerConfig_);
  robotCommand->spinOnce();

  EXPECT_EQ(1 + 1, 2);
}

TEST_F(RobotCommandTestFixture, RobotGoalCommandInit) {
  const std::string topic = "/move_base/simple/goal";
  std::unique_ptr<RobotCommand> robotCommand =
      std::make_unique<RobotGoalCommand>(nh, topic, robotName, plannerConfig_);
  robotCommand->spinOnce();

  EXPECT_EQ(1 + 1, 2);
}

TEST_F(RobotCommandTestFixture, RobotPlanCommandInit) {
  const std::string topic = "/move_base/simple/goal";
  std::unique_ptr<RobotCommand> robotCommand =
      std::make_unique<RobotPlanCommand>(nh, topic, robotName, plannerConfig_);

  EXPECT_EQ(1 + 1, 2);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "testing_multi_command_interface");

  return RUN_ALL_TESTS();
}