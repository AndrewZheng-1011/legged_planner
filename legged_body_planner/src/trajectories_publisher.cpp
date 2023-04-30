#include "legged_body_planner/trajectories_publisher.h"

// TODO (AZ): Make a abstract class
TrajectoriesPublisher::TrajectoriesPublisher(ros::NodeHandle nh,
                                             const std::string& topic_prefix)
    : tf2_(buffer_) {
  nh_ = nh;
  // Load params
  planning_utils::loadROSParam(nh_, "/trajectories_publisher/update_rate",
                               update_rate_);
  planner_config_.loadParams(nh_);

  // Trajectories publisher
  target_trajectories_publisher_.reset(
      new ocs2::TargetTrajectoriesRosPublisher(nh, topic_prefix));

  observation_sub_ = nh_.subscribe<ocs2_msgs::mpc_observation>(
      topic_prefix + "_mpc_observation", 1,
      &TrajectoriesPublisher::observationCallback, this);

  goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
      "/move_base_simple/goal", 1, &TrajectoriesPublisher::goalCallback, this);
  cmd_vel_sub_ = nh_.subscribe<geometry_msgs::Twist>(
      "/cmd_vel", 1, &TrajectoriesPublisher::cmdVelCallback, this);

  std::string body_plan_topic;
  planning_utils::loadROSParam(nh_, "topics/body_plan", body_plan_topic);
  body_plan_sub_ = nh_.subscribe<legged_body_msgs::Plan>(
      body_plan_topic, 1, &TrajectoriesPublisher::bodyPlanCallback, this);

  ROS_INFO("Initialized Trajectories Publisher");
};

ocs2::scalar_t TrajectoriesPublisher::estimateTimeToTarget(
    const ocs2::vector_t& pos1, const ocs2::vector_t& pos2) {
  const ocs2::vector_t difference_pos = pos2 - pos1;
  const ocs2::scalar_t& dx = difference_pos(0);
  const ocs2::scalar_t& dy = difference_pos(1);
  const ocs2::scalar_t& dyaw = difference_pos(3);
  const ocs2::scalar_t& rotation_time =
      std::abs(dyaw) / planner_config_.TARGET_ROTATION_VELOCITY;
  const ocs2::scalar_t& displacement = std::sqrt(dx * dx + dy * dy);
  const ocs2::scalar_t& displacement_time =
      displacement / planner_config_.TARGET_DISPLACEMENT_VELOCITY;
  return std::max(rotation_time, displacement_time);
}

ocs2::TargetTrajectories TrajectoriesPublisher::targetPoseToTargetTrajectories(
    const ocs2::vector_t& target_pose,
    const ocs2::SystemObservation& observation,
    const ocs2::scalar_t target_reaching_time) {
  // Desired time trajectory
  const ocs2::scalar_array_t time_trajectory{
      observation.time, target_reaching_time};  // Initiaize & declare
                                                // std::vector @ t0 and t_target
  // Desired state trajecotry
  ocs2::vector_t current_pose = observation.state.segment<6>(6);  // x,y,z,y,p,r
  current_pose(2) = planner_config_.COM_HEIGHT;
  current_pose(4) = 0;
  current_pose(5) = 0;

  // Create std::vector to contain initial and final state trajectory
  ocs2::vector_array_t state_trajectory(
      2, ocs2::vector_t::Zero(observation.state.size()));

  // State trajectory: x_dot, euler_rate, x, euler, and q's
  state_trajectory[0] << ocs2::vector_t::Zero(6), current_pose,
      planner_config_.DEFAULT_JOINT_STATES;
  state_trajectory[1] << ocs2::vector_t::Zero(6), target_pose,
      planner_config_.DEFAULT_JOINT_STATES;

  // Desired input traj (just get right dim, not used here)
  const ocs2::vector_array_t input_trajectory(
      2, ocs2::vector_t::Zero(observation.input.size()));
  return ocs2::TargetTrajectories(time_trajectory, state_trajectory,
                                  input_trajectory);
}

ocs2::TargetTrajectories TrajectoriesPublisher::goalToTargetTrajectories(
    const ocs2::vector_t& goal, const ocs2::SystemObservation& observation) {
  // Current position (x,ypr) x in R^3
  const ocs2::vector_t current_pose = observation.state.segment<6>(6);
  ocs2::vector_t target_pose(6);
  target_pose[0] = goal[0];                     // x goal
  target_pose[1] = goal[1];                     // y goal
  target_pose[2] = planner_config_.COM_HEIGHT;  // z goal
  target_pose[3] = goal[3];                     // yaw goal
  target_pose[4] = 0;                           // pitch goal
  target_pose[5] = 0;                           // roll goal

  const ocs2::scalar_t target_reaching_time =
      observation.time + estimateTimeToTarget(current_pose, target_pose);

  return targetPoseToTargetTrajectories(target_pose, observation,
                                        target_reaching_time);
}

ocs2::TargetTrajectories TrajectoriesPublisher::planToTargetTrajectories(
    const legged_body_msgs::Plan::ConstPtr& plan,
    const ocs2::SystemObservation& observation) {
  // Time
  ocs2::scalar_array_t time_trajectories(plan->times.begin(),
                                         plan->times.end());
  // std::cout << "Time trajectory: " << time_trajectories[0] << std::endl;
  // std::cout << time_trajectories[1] << std::endl;
  // std::cout << time_trajectories[2] << std::endl;

  const int default_joint_states_size =
      planner_config_.DEFAULT_JOINT_STATES.size();

  // States
  std::size_t N = plan->states.size();  // N : # of trajectory
  time_trajectories.resize(N);
  ocs2::vector_array_t state_trajectories(N);
  for (std::size_t i = 0; i < N; i++) {
    const std::vector<ocs2::scalar_t> state_val(
        plan->states[i].value.begin(),
        plan->states[i].value.end());  // Convert to double
    planning_utils::stdVecToEigen(state_val, state_trajectories[i]);
    state_trajectories[i].segment(planner_config_.NUM_STATES,
                                  default_joint_states_size) =
        planner_config_.DEFAULT_JOINT_STATES;  // Add default joint states
  }

  // Control
  N = plan->controls.size();
  ocs2::vector_array_t control_trajectories(N);
  for (std::size_t i = 0; i < N; i++) {
    const std::vector<ocs2::scalar_t> ctrl_val(
        plan->controls[i].value.begin(),
        plan->controls[i].value.end());  // Convert to double
    planning_utils::stdVecToEigen(ctrl_val, control_trajectories[i]);
  }

  return ocs2::TargetTrajectories(time_trajectories, state_trajectories,
                                  control_trajectories);
}

ocs2::TargetTrajectories TrajectoriesPublisher::cmdVelToTargeTrajectories(
    const ocs2::vector_t& cmd_vel, const ocs2::SystemObservation& observation) {
  // Get current state
  const ocs2::vector_t current_pose = observation.state.segment<6>(6);

  // Get rotate cmd_vel
  const Eigen::Matrix<ocs2::scalar_t, 3, 1> ypr =
      current_pose.tail(3);  // YPR of euler angles
  ocs2::vector_t cmd_vel_rot = ocs2::getRotationMatrixFromZyxEulerAngles(ypr) *
                               cmd_vel.head(3);  // cmd vel w.r.t. body frame
  const ocs2::scalar_t time_to_target = planner_config_.PLAN_HORIZON;

  // Define target pose
  ocs2::vector_t target(6);
  target(0) = current_pose(0) + cmd_vel_rot(0) * time_to_target;
  target(1) = current_pose(1) + cmd_vel_rot(1) * time_to_target;
  target(2) = planner_config_.COM_HEIGHT;
  target(3) = current_pose(3) + cmd_vel(3) * time_to_target;  // yaw
  target(4) = 0;
  target(5) = 0;

  const ocs2::vector_t target_pose = target;

  // Target reaching time
  const ocs2::scalar_t target_reaching_time =
      observation.time + time_to_target;  // Fixed time
  auto trajectories = targetPoseToTargetTrajectories(target_pose, observation,
                                                     target_reaching_time);

  // Set initial and final velocity to be the following
  trajectories.stateTrajectory[0].head(3) = cmd_vel_rot;
  trajectories.stateTrajectory[1].head(3) = cmd_vel_rot;
  return trajectories;
}

void TrajectoriesPublisher::observationCallback(
    const ocs2_msgs::mpc_observation::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(latest_observation_mutex_);
  latest_observation_ = ocs2::ros_msg_conversions::readObservationMsg(*msg);
}

void TrajectoriesPublisher::goalCallback(
    const geometry_msgs::PoseStamped::ConstPtr& msg) {
  if (latest_observation_.time == 0) {
    return;
  }
  geometry_msgs::PoseStamped pose = *msg;  // Dereference the msg

  try {
    buffer_.transform(
        pose, pose, "odom",
        ros::Duration(0.2));  // Simply putting pose msg into odom?
  } catch (tf2::TransformException& ex) {
    ROS_WARN("Failure %s\n", ex.what());
    return;
  }

  //  cmd_goal is x,y,z, yaw, pitch, roll
  ocs2::vector_t cmd_goal = ocs2::vector_t::Zero(6);
  cmd_goal[0] = pose.pose.position.x;
  cmd_goal[1] = pose.pose.position.y;
  cmd_goal[2] = pose.pose.position.z;

  // Use Eigen to convert to euler angles
  Eigen::Quaternion<ocs2::scalar_t> q(
      pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y,
      pose.pose.orientation.z);

  cmd_goal[3] = q.toRotationMatrix().eulerAngles(0, 1, 2).z();
  cmd_goal[4] = q.toRotationMatrix().eulerAngles(0, 1, 2).y();
  cmd_goal[5] = q.toRotationMatrix().eulerAngles(0, 1, 2).x();

  const auto trajectories =
      goalToTargetTrajectories(cmd_goal, latest_observation_);
  target_trajectories_publisher_->publishTargetTrajectories(trajectories);
}

void TrajectoriesPublisher::cmdVelCallback(
    const geometry_msgs::Twist::ConstPtr& msg) {
  if (latest_observation_.time == 0.0) {
    // std::cout << "Latest observation is: " << latest_observation_.time
    //           << std::endl;
    return;
  }

  // std::cout << "Past observation 0" << std::endl;
  ocs2::vector_t cmd_vel = ocs2::vector_t::Zero(4);
  cmd_vel[0] = msg->linear.x;
  cmd_vel[1] = msg->linear.y;
  cmd_vel[2] = msg->linear.z;
  cmd_vel[3] = msg->angular.z;

  const auto trajectories =
      cmdVelToTargeTrajectories(cmd_vel, latest_observation_);
  target_trajectories_publisher_->publishTargetTrajectories(trajectories);
}

void TrajectoriesPublisher::bodyPlanCallback(
    const legged_body_msgs::Plan::ConstPtr& msg) {
  if (latest_observation_.time == 0.0) {
    return;
  }
  const auto trajectories = planToTargetTrajectories(msg, latest_observation_);
  target_trajectories_publisher_->publishTargetTrajectories(trajectories);
}

void TrajectoriesPublisher::spin() {
  // Load parameters
  std::cout << "update rate: " << update_rate_ << std::endl;
  ros::Rate r(update_rate_);
  while (ros::ok()) {
    // Process callbacks
    ROS_WARN_THROTTLE(1, "SPINNING IN TRAJ PUBLISHER");
    ros::spinOnce();

    // Publish to target trajectories

    // Publish if results are valid
    r.sleep();
  }
}

void TrajectoriesPublisher::spinOnce() { ros::spinOnce(); }
