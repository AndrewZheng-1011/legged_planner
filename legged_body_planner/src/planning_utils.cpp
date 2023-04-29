#include "legged_body_planner/planning_utils.h"

namespace planning_utils {

bool checkTrajectoriesAppropriateSize(
    const legged_body_msgs::Plan::ConstPtr& plan) {
  if (plan->states.empty() || plan->controls.empty() || plan->times.empty()) {
    ROS_ERROR("State, control, and/or time trajectory is empty");
    return false;
  }

  std::size_t state_traj_size = plan->states.size();
  std::size_t ctrl_traj_size = plan->controls.size();
  std::size_t time_traj_size = plan->times.size();

  //   std::cout << "state_traj_size: " << state_traj_size << std::endl;
  //   std::cout << "ctrl_traj_size: " << ctrl_traj_size << std::endl;
  //   std::cout << "time_traj_size: " << time_traj_size << std::endl;

  // TODO (AZ): Checks if this works when no publishing
  // Checks that state, control, and time trajectory have appropriate size
  if (!(state_traj_size == time_traj_size &&
        state_traj_size == ctrl_traj_size)) {
    ROS_ERROR("State, control, and/or time trajectory size do not match up!");
    return false;
  }
  return true;
}

void stateToRigidBodyState(const std::vector<float>& state,
                           std::vector<float>& rigid_body_state,
                           const PlannerConfig& planner_config) {
  // TODO (AZ): Improvise method for pitch etc. & make this virtual function
  // s.t. up to user implementation
  // TODO (AZ): Enforce if heading angle assumption

  // Since rigid body planer is 12 states, resize
  rigid_body_state.resize(planner_config.NUM_STATES +
                          planner_config.DEFAULT_JOINT_STATES.size());

  // Ensure that when transferring states to rigid body states, do not extend
  // pass rigid body state size
  int num_states;
  if (state.size() <= planner_config.NUM_STATES) {
    num_states = state.size();
  } else {
    num_states = planner_config.NUM_STATES;
  }

  // Rigid Body States in the following order:
  // x_dot, y_dot, z_dot, yaw_dot, pitch_dot, roll_dot, x, y, z, r, p, y
  for (int i = 0; i < num_states; i++) {
    rigid_body_state[i] = state[i];
  }

  // Enforce robot parameters
  rigid_body_state[4] = 0;  // pitch dot
  rigid_body_state[5] = 0;  // roll dot
  rigid_body_state[8] =
      planner_config.COM_HEIGHT;  // z | Use terrain info later
  rigid_body_state[10] = 0;       // pitch | Use terrain info later
  rigid_body_state[11] = 0;       // roll | Use terrain info later
}

void controlToRigidBodyControl(const std::vector<float>& control,
                               std::vector<float>& rigid_body_control,
                               ocs2::SystemObservation& observation,
                               const PlannerConfig& planner_config) {
  // TODO (AZ): Test what changing control values do
  // Set all to zero currently
  // const int RIGID_BODY_CONTROL_DIM =
  //     observation.input.size();  // This only work if observation is
  //     observing
  const int RIGID_BODY_CONTROL_DIM = planner_config.NUM_CONTROLS;
  rigid_body_control.resize(RIGID_BODY_CONTROL_DIM);
  std::fill(rigid_body_control.begin(), rigid_body_control.end(), 0.0);
}

}  // namespace planning_utils