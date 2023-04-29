#include <gtest/gtest.h>
#include <ros/package.h>
#include <ros/ros.h>

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

#include "legged_body_planner/legged_body_planner.h"
#include "legged_body_planner/planning_utils.h"

TEST(TestLeggedBodyPlanner, InitLeggedBodyPlannerObject) {
  ros::NodeHandle nh;
  std::string robot_name = "legged_robot";  // topic_prefix
  LeggedBodyPlanner legged_body_planner(nh, robot_name);
  EXPECT_EQ(1 + 1, 2) << "Did not initialize legged body planner";
}

TEST(TestLeggedBodyPlanner, PublishStatesToRigidBodyStates) {
  // TODO (AZ) : Make test independent of robot type etc, that way, I don't need
  // assert command

  ros::NodeHandle nh;
  const std::string robot_name = "legged_robot";
  const std::string robot_type = "go1";

  // Get parameters
  const std::string reference_file =
      ros::package::getPath("legged_body_planner") + "/config/" + robot_type +
      "/reference.info";
  const std::string& task_file = ros::package::getPath("legged_body_planner") +
                                 "/config/" + robot_type + "/task.info";

  boost::filesystem::path task_file_path(task_file);
  ASSERT_TRUE(boost::filesystem::exists(task_file_path))
      << "Task files not found w/ command input";

  boost::filesystem::path reference_file_path(reference_file);
  ASSERT_TRUE(boost::filesystem::exists(reference_file_path))
      << "Task files not found w/ command input";

  LeggedBodyPlanner legged_body_planner(nh, robot_name);
  planning_utils::PlannerConfig planner_config;
  planner_config.loadParams(nh);

  // Create dummy plan
  legged_body_msgs::Plan generic_plan;
  generic_plan.states.resize(4);
  generic_plan.controls.resize(4);
  generic_plan.times.resize(4);

  for (int i = 0; i < 4; i++) {
    generic_plan.times[i] = i;
    // Generic plan w/ smaller states than rigid body
    generic_plan.states[i].value = {0, 0, 0, 0, 0, 0};
    generic_plan.controls[i].value = {0, 0, 0, 0, 0, 0};
  }

  const auto generic_plan_ptr =
      boost::make_shared<const legged_body_msgs::Plan>(generic_plan);

  legged_body_msgs::Plan rigid_body_plan;

  // Check Rigid Body Plan function
  bool plan_to_body_plan = legged_body_planner.planToRigidBodyPlan(
      generic_plan_ptr, rigid_body_plan, planner_config);
  EXPECT_TRUE(plan_to_body_plan) << "Conversion to plan to rigid body plan for "
                                    "12 to 12 states not working...";

  // Check values are correct
  for (int i = 0; i < 4; i++) {
    // Check time
    EXPECT_EQ(rigid_body_plan.times[i], i);

    // Check states
    for (int j = 0; j < planner_config.NUM_STATES; j++) {
      if (j == 8) {  // z component
        EXPECT_FLOAT_EQ(rigid_body_plan.states[i].value[j],
                        planner_config.COM_HEIGHT);
      } else {
        EXPECT_FLOAT_EQ(rigid_body_plan.states[i].value[j], 0);
      }
    }
    // Check control
    for (int j = 0; j < planner_config.NUM_CONTROLS; j++) {
      EXPECT_FLOAT_EQ(rigid_body_plan.controls[i].value[j], 0);
    }
  }

  // TODO (AZ): Test out this works... when size don't match, etc.
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "legged_body_planner_tester");
  return RUN_ALL_TESTS();
}
