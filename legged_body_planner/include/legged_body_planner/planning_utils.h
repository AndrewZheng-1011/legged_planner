#ifndef PLANNING_UTILS_H
#define PLANNING_UTILS_H

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ros/ros.h>

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

#include "legged_body_msgs/Control.h"
#include "legged_body_msgs/Plan.h"
#include "legged_body_msgs/State.h"

namespace planning_utils {

/**
 * @brief Load ROS parameter into class variable
 * @param nh ROS nodehandle
 * @param param_name String storing key of param in ROS parameter server
 * @param var_name Address of variable to store value associated w/ key
 * @return true Successfully retrieved value from key in param server
 * @return false Unsuccessful retrieval of value from key in param server
 */
template <typename ParamType>
inline bool loadROSParam(ros::NodeHandle& nh, std::string param_name,
                         ParamType& var_name) {
  if (!nh.getParam(param_name, var_name)) {
    ROS_ERROR("Can't find param %s from parameter server", param_name.c_str());
    return false;
  }
  return true;
}

/**
 * @brief Converts Eigen to std::vector
 * @param state_eig Eigen vector
 * @param state_vec std::vector
 */
template <typename T>
void eigenToStdVec(const Eigen::Matrix<T, Eigen::Dynamic, 1>& eig,
                   std::vector<T>& vec) {
  // Optimize this later
  vec.resize(eig.size());
  for (std::size_t i = 0; i < eig.size(); i++) {
    vec[i] = eig[i];
  }
}

/**
 * @brief Converts std::vector to Eigen
 * @tparam T Datatype of vectors/eigen
 * @param state_vec Vector
 * @param state_eig Eigen vector
 */
template <typename T>
void stdVecToEigen(const std::vector<T>& vec,
                   Eigen::Matrix<T, Eigen::Dynamic, 1>& eig) {
  // Optimize this later
  eig.resize(vec.size());
  for (std::size_t i = 0; i < vec.size(); i++) {
    eig[i] = vec[i];
  }
}

/**
 * @brief Planner Configuration for Legged Robot
 */
struct PlannerConfig {
  // Robot parameters
  double COM_HEIGHT;
  double PLAN_HORIZON;
  double NUM_STATES;
  double NUM_CONTROLS;
  double TARGET_ROTATION_VELOCITY;
  double TARGET_DISPLACEMENT_VELOCITY;
  double TIME_TO_TARGET;
  Eigen::Matrix<double, 12, 1>
      DEFAULT_JOINT_STATES;  // TODO (AZ): Get state dim later

  // Planning parameters
  double dt;                             // Discretization of trajectory
  std::vector<float> goal_state_vector;  // Goal State of planner
  std::string frame_id;                  // Frame ID of planner

  // Files for parameters
  std::string reference_file;  // Robot reference file
  std::string task_file;       // Robot tasks file

  void loadParams(ros::NodeHandle& nh) {
    loadParamsFromServer(nh);
    loadParamsFromInfoFile();
  }

 private:
  void loadParamsFromServer(ros::NodeHandle& nh) {
    planning_utils::loadROSParam(nh, "/referenceFile", reference_file);
    planning_utils::loadROSParam(nh, "/taskFile", task_file);
    planning_utils::loadROSParam(nh, "/legged_body_planner/dt", dt);
    planning_utils::loadROSParam(nh, "/legged_body_planner/goal_state",
                                 goal_state_vector);
    planning_utils::loadROSParam(nh, "/legged_body_planner/frame_id", frame_id);
    planning_utils::loadROSParam(nh, "/legged_body_planner/num_states",
                                 NUM_STATES);
    planning_utils::loadROSParam(nh, "/legged_body_planner/num_controls",
                                 NUM_CONTROLS);
  }

  void loadParamsFromInfoFile() {
    boost::filesystem::path reference_file_path(reference_file);
    if (boost::filesystem::exists(reference_file_path)) {
      std::cerr << "[LeggedPlanningInterface] Loading reference file: "
                << reference_file_path << std::endl;
    } else {
      throw std::invalid_argument(
          "[LeggedPlanningInterface] Reference file path not found: " +
          reference_file_path.string());
    }

    boost::filesystem::path task_file_path(task_file);
    if (boost::filesystem::exists(task_file_path)) {
      std::cerr << "[LeggedPlanningInterface] Loading task file: "
                << task_file_path << std::endl;
    } else {
      throw std::invalid_argument(
          "[LeggedPlanningInterface] Task file path not found: " +
          reference_file_path.string());
    }
    ocs2::loadData::loadCppDataType(reference_file, "comHeight", COM_HEIGHT);
    ocs2::loadData::loadEigenMatrix(reference_file, "defaultJointState",
                                    DEFAULT_JOINT_STATES);
    ocs2::loadData::loadCppDataType(task_file, "mpc.timeHorizon", PLAN_HORIZON);
    ocs2::loadData::loadCppDataType(reference_file, "targetRotationVelocity",
                                    TARGET_ROTATION_VELOCITY);
    ocs2::loadData::loadCppDataType(reference_file,
                                    "targetDisplacementVelocity",
                                    TARGET_DISPLACEMENT_VELOCITY);
  }
};

/**
 * @brief Check if trajectories are appropriately sized
 * @param plan Plan to check state, control, and time trajectories
 * @return true If trajectories are appropriately sized
 * @return false If trajectories are not appropriately sized
 */
bool checkTrajectoriesAppropriateSize(
    const legged_body_msgs::Plan::ConstPtr& plan);

/**
 * @brief Converts a vector of states to rigid body states
 * @param state Generic states
 * @param rigid_body_state Rigid body state
 * @param planner_config Configurations to help for robot specific conversions
 */
void stateToRigidBodyState(const std::vector<float>& state,
                           std::vector<float>& rigid_body_state,
                           const PlannerConfig& planner_config);

/**
 * @brief Converts control from generic plan to rigid body control
 * @param control Generic plan control
 * @param rigid_body_control Rigid body control
 */
void controlToRigidBodyControl(const std::vector<float>& control,
                               std::vector<float>& rigid_body_control,
                               ocs2::SystemObservation& observation,
                               const PlannerConfig& planner_config);

}  // namespace planning_utils

#endif
