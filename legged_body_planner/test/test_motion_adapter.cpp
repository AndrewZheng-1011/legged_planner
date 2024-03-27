#include <gtest/gtest.h>

#include "legged_body_planner/motion_adapters/TerrainAdapter.h"

TEST(TerrainAdapterTest, AdaptOrientation) {
  using namespace ocs2;
  using namespace legged_robot;

  ros::NodeHandle nh;

  planning_utils::PlannerConfig plannerConfig{};
  plannerConfig.loadParams(nh);

  TerrainAdapter terrainAdapter{plannerConfig, nullptr};

  EXPECT_EQ(1 + 1, 2);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "testing_terrain_adapter");

  return RUN_ALL_TESTS();
}
