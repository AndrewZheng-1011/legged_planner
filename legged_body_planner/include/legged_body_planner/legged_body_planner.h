#ifndef LEGGED_BODY_PLANNER_H
#define LEGGED_BODY_PLANNER_H

#include <legged_body_msgs/Control.h>
#include <legged_body_msgs/Plan.h>
#include <legged_body_msgs/State.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ros/ros.h>

#include <mutex>

#include "legged_body_planner/planning_utils.h"
#include "legged_body_planner/trajectories_publisher.h"

//! Body Planning class for legged robots
/*!
  LeggedBodyPlanner is a container for logic utilized in legged body planner
  node. This algorithm requires terrain information (future work as GridMap
  message type), start state, and goal state for rigid body. The class will then
  subscribe to planning algorithm and convert generic plan to legged rigid body
  model plan. The plan will publish these state trajectories in the form of
  discretized states.
*/
class LeggedBodyPlanner {
 public:
  /**
   * @brief Construct a new Legged Body Planner object
   *
   */
  LeggedBodyPlanner(ros::NodeHandle& nh, std::string topic_prefix);

  /**
   * @brief Primary work function in class. Called in node file for this
   * component
   */
  void spin();

  /**
   * @brief Converts generic plan to rigid body plan | Remark : future virtual
   * function | TODO (AZ): Create plan class and make this public there
   * @param plan Generic plan
   * @param rigid_body_plan Plan for rigid body
   * @param planner_config PlannerConfig containing robot and planning params
   * @return true If plan to rigid body plan was successful
   * @return false If plan to rigid body plan was unsuccessful (i.e.. plan
   * trajectories are not the same)
   */
  bool planToRigidBodyPlan(const legged_body_msgs::Plan::ConstPtr& plan,
                           legged_body_msgs::Plan& rigid_body_plan,
                           const planning_utils::PlannerConfig& planner_config);

 private:
  /**
   * @brief Function to read state trajectory from a file
   */
  void readStateTrajectories();

  /**
   * @brief Get current observation
   * @param msg Observer message
   */
  void observerCallback(const ocs2_msgs::mpc_observation::ConstPtr& msg);

  /**
   * @brief Subscribes to generic planner and converts to rigid body plan
   * @param plan Msg from generic planner
   */
  void planCallback(const legged_body_msgs::Plan::ConstPtr& plan);

  /**
   * @brief Terrain callback (TODO (AZ): probably a service)
   */
  void terrainCallback();

  /**
   * @brief Publish current plan
   */
  void publishCurrentPlan();

  /// @brief Nodehandle
  ros::NodeHandle nh_;

  /// Subscriber for planning algorithm
  ros::Subscriber planning_algorithm_sub_;

  /// @brief Subscriber for goal state
  ros::Subscriber goal_configuration_sub_;

  /// @brief Publisher for body plan messages
  ros::Publisher body_plan_pub_;

  /// @brief Mutex on observer
  mutable std::mutex latest_observation_mutex_;

  /// @brief Observer of the system
  ocs2::SystemObservation latest_observation_;

  /// @brief Planner Configurations
  planning_utils::PlannerConfig planner_config_;

  // /// @brief Shared pointer for object which converts msg to target
  // trajectories
  std::unique_ptr<TrajectoriesPublisher> trajectories_publisher_ptr_;

  /// @brief Current robot configuration (x_dot, euler rate, x, euler)
  legged_body_msgs::State rigid_body_states_;

  /// @brief Goal robot configuration (2d?)
  legged_body_msgs::State goal_state_;

  /// @brief Control of rigid body
  legged_body_msgs::Control control_;

  // Plan

  /// @brief Plan of rigid body
  legged_body_msgs::Plan body_plan_;  // TODO (AZ): May not need this, delete

  /// @brief ID for status of planner
  int planner_status_;

  /// @brief Time of planner
  ros::Time plan_time_;  // TODO (AZ) : Do I want ros time?

  /// @brief Delay for publishing new plan
  bool plan_pub_delay_;

  /// @brief If planner replans
  bool replan_;

  /// @brief Classification of first plan vs other plans
  bool first_plan_;

  /// @brief Boolean to classify successful retrieval of plan
  bool retrieved_plan_;

  /// @brief Boolean to terminate planner
  bool terminate_planner_;

  // Body Plan
  const int NUM_STATES_ = 12;

  /// @brief Update rate of planner
  double update_rate_;
};
#endif  // LEGGED_BODY_PLANNER_H
