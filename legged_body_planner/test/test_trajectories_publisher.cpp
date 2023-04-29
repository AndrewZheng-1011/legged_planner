#include <gtest/gtest.h>
#include <ros/ros.h>

#include "legged_body_planner/trajectories_publisher.h"

TEST(TestTrajectoriesPublisher, InitTrajectoriesPublisher) {
  ros::NodeHandle nh;
  const std::string robot_name = "legged_robot";

  // Call default constructor
  TrajectoriesPublisher trajectories_publisher(nh, robot_name);
  EXPECT_EQ(1 + 1, 2);  // << "1 + 1 is not equal to 2";
}

TEST(TestTrajectoriesPublisher, EigenConversion) {
  std::cout << "Try to convert to eigen\n";

  // Convert to Eigen
  ocs2::vector_t foo(5);
  foo << 1.0, 2.0, 3.0, 4.0, 5.0;
  ocs2::vector_t bar = Eigen::Map<Eigen::Matrix<double, 5, 1>>(foo.data());

  EXPECT_EQ(1 + 1, 2);
}

int main(int argc, char **argv) {
  // Initialize the google test object
  testing::InitGoogleTest(&argc, argv);  // Memory address of argc

  // Initialize ROS node
  ros::init(argc, argv, "testing_trajectories_publisher");

  // Run Gtest Macro
  return RUN_ALL_TESTS();
}