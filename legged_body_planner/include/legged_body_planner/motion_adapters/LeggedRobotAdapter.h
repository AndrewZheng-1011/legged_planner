#ifndef LEGGED_ROBOT_ADAPTER_H
#define LEGGED_ROBOT_ADAPTER_H

#include <legged_body_utils/planning_utils.h>

#include "legged_body_planner/motion_adapters/MotionAdapter.h"

class LeggedRobotAdapter : public MotionAdapter {
 public:
  LeggedRobotAdapter(planning_utils::PlannerConfig& plannerConfig);

  void adaptMotion(ocs2::scalar_array_t& times,
                   ocs2::vector_array_t& stateTrajectories,
                   ocs2::vector_array_t& controlTrajectories) override;

 protected:
  planning_utils::PlannerConfig plannerConfig_;

 private:
  template <typename ArrayType1, typename ArrayType2>
  bool checkEqualSize(ArrayType1 array1, ArrayType2 array2) {
    if (!(array1.size() == array2.size())) {
      std::cerr << "[LeggedRobotAdapter] Array sizes are not equal!"
                << std::endl;
      return false;
    }
    return true;
  }

  /**
   * @brief Mainly a method to resize to legged state size. Does enforce height
   * constraint and default joint states
   * @param state
   * @return ocs2::vector_t
   */
  ocs2::vector_t toLeggedState(const ocs2::vector_t& state);

  /**
   * @brief Mainly a method to resize to legged control state.
   *
   * @param control
   * @return ocs2::vector_t
   */
  ocs2::vector_t toLeggedControl(const ocs2::vector_t& control);
};

#endif  // LEGGED_ROBOT_ADAPTER_H
