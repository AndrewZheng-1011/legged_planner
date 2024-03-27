#ifndef ROBOT_COMMAND_H
#define ROBOT_COMMAND_H

#include <ocs2_core/Types.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>
#include <ros/ros.h>

/*!
Abstract class that takes in a robot input (e.g. velocity command, goal
positions, or robot plan) and the current robot observed state, and translates
that command to suitable legged robot trajectories (in the form of ocs2 target
trajectories)

ROS component:
- Subscribes to a robot command topic
- Publishes

TODO (AZ):
- Separate ROS and core algorithm
- Maybe enforce a structure s.t. get vectors first before target trajectories...
this enforces that we can use a motion adapter
*/
class RobotCommand {
 public:
  RobotCommand(ros::NodeHandle& nh, const std::string& robotCommandTopic,
               const std::string& topicPrefix);

  virtual ~RobotCommand() = default;

  /**
   * @brief TODO (AZ): Need asynchronous processing for callbacks
   *
   */
  virtual void spinOnce();

 protected:
  // TODO (AZ): May have to do template
  virtual ocs2::TargetTrajectories robotInputToTargetTrajectories(
      const ocs2::vector_t& input,
      const ocs2::SystemObservation& observation) = 0;

  // virtual ocs2::TargetTrajectories robotInputsToTargetTrajectories(
  //     const ocs2::vector_array_t inputs,
  //     const ocs2::SystemObservation& observation);

  ocs2::TargetTrajectories targetTrajectories_;
  ocs2::SystemObservation latestObservation_;
  ros::Subscriber observationSub_;
  std::mutex latestObservationMutex_;

  /// @brief  Robot command subscriber
  ros::Subscriber
      sub_;  // TODO (AZ): Maybe move it down as difficult to
             // implement generic sub -> No need for robot command topic

  /// @brief Trajectory to be executed
  std::unique_ptr<ocs2::TargetTrajectoriesRosPublisher>
      targetTrajectoriesPublisherPtr_;

  double updateRate_;
};

#endif  // ROBOT_COMMAND_H