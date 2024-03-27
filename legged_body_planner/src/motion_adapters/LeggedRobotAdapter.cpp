#include "legged_body_planner/motion_adapters/LeggedRobotAdapter.h"

LeggedRobotAdapter::LeggedRobotAdapter(
    planning_utils::PlannerConfig& plannerConfig) {
  plannerConfig_ = plannerConfig;
}

void LeggedRobotAdapter::adaptMotion(
    ocs2::scalar_array_t& times, ocs2::vector_array_t& stateTrajectories,
    ocs2::vector_array_t& controlTrajectories) {
  // Check that trajectory equal sized -> If not, do not modify
  if (!checkEqualSize(times, stateTrajectories) &&
      !checkEqualSize(times, stateTrajectories)) {
    return;
  }
  // Modify time to legged time (if need to be)

  // Modify state to legged states
  std::size_t N = stateTrajectories.size();
  for (std::size_t i = 0; i < N; i++) {
    stateTrajectories.at(i) = toLeggedState(
        stateTrajectories.at(i));  // TODO: Change w/ pass by reference instead
  }

  // Modify control to legged controls
  N = controlTrajectories.size();
  for (std::size_t i = 0; i < N; i++) {
    controlTrajectories.at(i) = toLeggedControl(controlTrajectories.at(i));
  }
}

ocs2::vector_t LeggedRobotAdapter::toLeggedState(const ocs2::vector_t& state) {
  ocs2::vector_t leggedState{plannerConfig_.NUM_STATES +
                             plannerConfig_.DEFAULT_JOINT_STATES.size()};
  leggedState.setZero();
  // Assumption: Assumes states are in order from top to bottom
  int numStates;
  if (state.size() <= plannerConfig_.NUM_STATES) {
    numStates = state.size();
  } else {
    numStates = plannerConfig_.NUM_STATES;
  }

  // Rigid Body States in the following order:
  // x_dot, y_dot, z_dot, yaw_dot, pitch_dot, roll_dot, x, y, z, y, p, r
  // Note: ypr in body frame of reference (not body aligned frame)
  for (int i = 0; i < numStates; i++) {
    leggedState[i] = state[i];
  }
  leggedState[8] = plannerConfig_.COM_HEIGHT;  // z

  leggedState.segment(plannerConfig_.NUM_STATES,
                      plannerConfig_.DEFAULT_JOINT_STATES.size()) =
      plannerConfig_.DEFAULT_JOINT_STATES;

  return leggedState;
}

ocs2::vector_t LeggedRobotAdapter::toLeggedControl(
    const ocs2::vector_t& control) {
  // TODO (AZ): Modify control values later | Note: control is q_v and GRFs
  ocs2::vector_t leggedControl{plannerConfig_.NUM_CONTROLS};
  leggedControl.setZero();

  return leggedControl;
}