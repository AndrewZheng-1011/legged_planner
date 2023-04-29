#ifndef TRAJECTORIES_PUBLISHER_H
#define TRAJECTORIES_PUBLSIHER_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <mutex>

#include "legged_body_msgs/Control.h"
#include "legged_body_msgs/Plan.h"
#include "legged_body_msgs/State.h"
#include "legged_body_planner/planning_utils.h"

//! Class that subscribes to topics and publishes the topics as
//! TargetTrajectories
/*!
  TrajectoriesPublisher is a class that subscribes to topics and transforms the
  following ROS msgs to TargetTrajectories. The class is capable of converting
  body_plan msgs to TargetTrajectories

  Topics subscribed:
  - /cmd_vel
  - /move_base/simple/goal
  - /plan
  ROS msgs conversion:
  - Twist -> TargetTrajectories
  - PoseStamped -> TargetTrajectories
  - BodyPlan -> TargetTrajectories
*/
class TrajectoriesPublisher {
 public:
  using CmdToTargetTrajectories_t = std::function<ocs2::TargetTrajectories(
      const ocs2::vector_t& cmd, const ocs2::SystemObservation& observation)>;
  using PlanToTargetTrajectories_t = std::function<ocs2::TargetTrajectories(
      const legged_body_msgs::Plan::ConstPtr& plan,
      const ocs2::SystemObservation& observation)>;

  /**
   * @brief Construct a new Trajectories Publisher object
   * @param nh ros nodehandle
   * @param topicPrefix prefix topic
   */
  TrajectoriesPublisher(ros::NodeHandle nh, const std::string& topic_prefix);

  /**
   * @brief Construct a new Trajectories Publisher object
   * @param nh ros nodehandle
   * @param topic_prefix prefix topic
   * @param goal_to_target_trajectories Function reference from goal to target
   * trajectories
   * @param cmd_vel_to_target_trajectories Function reference from cmd_vel to
   * target trajectories
   */
  TrajectoriesPublisher(
      ros::NodeHandle& nh, const std::string topic_prefix,
      CmdToTargetTrajectories_t goalToTargetTrajectories,
      CmdToTargetTrajectories_t cmdVelToTargetTrajectories/*,
      PlanToTargetTrajectories_t planToTargetTrajectories*/);

  // /**
  //  * @brief Converts plan to a target trajectories
  //  * @param plan Legged Body Msg plan
  //  * @return ocs2::TargetTrajectories TargetTrajectory
  //  */
  // ocs2::TargetTrajectories planToTargetTrajectories(
  //     const legged_body_msgs::Plan::ConstPtr& plan);

  /**
   * @brief Primary work function in class. Called in node file for this
   * component
   */
  void spin();

  /**
   * @brief Secondary work function in class. Spin once option for trajectories
   * publisher
   */
  void spinOnce();

  /**
   * @brief Estimate time to target with nominal target velocity
   * @param pos1 Position 1
   * @param pos2 Position 2
   * @return ocs2::scalar_t Return ideal time to target
   */
  ocs2::scalar_t estimateTimeToTarget(const ocs2::vector_t& pos1,
                                      const ocs2::vector_t& pos2);

  /**
   * @brief Converts pose to target trajectories
   * @param target_pose Eigen vector of poses
   * @param observation System Observation
   * @param target_reaching_time Time to reach target
   * @return ocs2::TargetTrajectories
   */
  ocs2::TargetTrajectories targetPoseToTargetTrajectories(
      const ocs2::vector_t& target_pose,
      const ocs2::SystemObservation& observation,
      const ocs2::scalar_t target_reaching_time);

  /**
   * @brief Helper function that converts goal to trajectories w/ a fixed
   * velocity
   * @param goal Target position
   * @param observation System observation
   * @return ocs2::TargetTrajectories Target Trajectories for Legged Robot
   */
  ocs2::TargetTrajectories goalToTargetTrajectories(
      const ocs2::vector_t& goal, const ocs2::SystemObservation& observation);

  /**
   * @brief Helper function that converts goal to velocity commands
   * @param cmd_vel Velocity command
   * @param observation System observation
   * @return ocs2::TargetTrajectories Target Trajectories for Legged Robot
   */
  ocs2::TargetTrajectories cmdVelToTargeTrajectories(
      const ocs2::vector_t& cmd_vel,
      const ocs2::SystemObservation& observation);

  /**
   * @brief Helper function that converts plan msg to Target Trajectories
   * @param plan Legged Robot Plan msg
   * @param observation System Observation
   * @return ocs2::TargetTrajectories Target Trajectories for Legged Robot
   */
  ocs2::TargetTrajectories planToTargetTrajectories(
      const legged_body_msgs::Plan::ConstPtr& plan,
      const ocs2::SystemObservation& observation);

 private:
  /**
   * @brief Observation subscriber
   * @param msg ocs2 mpc observation msg
   */
  void observationCallback(const ocs2_msgs::mpc_observation::ConstPtr& msg);

  /**
   * @brief Subscribes to goal point msgs to convert to target trajectories
   * @param msg Pose stamped geometry msg for goal location
   */
  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

  /**
   * @brief Subscribes to cmd_vel and publishes new velocity to target
   * trajectories
   * @param msg Twist msg for new location
   */
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);

  /**
   * @brief Subscribes to body plan to convert to target trajectories
   * @param msg
   */
  void bodyPlanCallback(const legged_body_msgs::Plan::ConstPtr& msg);

  /// TF2 Buffer
  tf2_ros::Buffer buffer_;

  /// Transform listener
  tf2_ros::TransformListener tf2_;

  /// Nodehandle
  ros::NodeHandle nh_;

  /// Subscribers
  ros::Subscriber observation_sub_, goal_sub_, cmd_vel_sub_, body_plan_sub_;

  /// Publisher
  std::unique_ptr<ocs2::TargetTrajectoriesRosPublisher>
      target_trajectories_publisher_;

  /// Mutex on Observer
  mutable std::mutex latest_observation_mutex_;

  /// @brief Observer for states
  ocs2::SystemObservation latest_observation_;

  /// @brief Planner Configuration | TODO (AZ): Delete s.t. not reusing
  planning_utils::PlannerConfig planner_config_;

  /// @brief Update rate for sending and receiving data
  double update_rate_;
};

#endif  // TRAJECTORIES_PUBLISHER_H
