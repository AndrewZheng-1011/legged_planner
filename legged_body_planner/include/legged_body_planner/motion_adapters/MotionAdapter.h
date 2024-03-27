#ifndef MOTION_ADAPTER_H
#define MOTION_ADAPTER_H

#include <ocs2_core/Types.h>

/**
 * @brief Abstract class to adapt trajectories for a system
 */
class MotionAdapter {
 public:
  MotionAdapter() = default;

  virtual ~MotionAdapter() = default;

  virtual void adaptMotion(ocs2::scalar_array_t& times,
                           ocs2::vector_array_t& stateTrajectories,
                           ocs2::vector_array_t& controlTrajectories) = 0;
};

#endif  // MOTION_ADAPTER_H