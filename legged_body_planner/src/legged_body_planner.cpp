#include "legged_body_planner/legged_body_planner.h"

LeggedBodyPlanner::LeggedBodyPlanner(ros::NodeHandle& nh,
                                     std::string topic_prefix) {
  // std::cout << "Initialized LeggedBodyPlanner class\n";

  nh_ = nh;
  // Get parameters
  std::string reference_file, task_file, plan_topic, body_plan_topic;
  std::vector<float> goal_state_vector(12);
  planning_utils::loadROSParam(nh_, "/referenceFile", reference_file);
  planning_utils::loadROSParam(nh_, "/taskFile", task_file);

  // Load rosparams
  planning_utils::loadROSParam(nh_, "topics/plan", plan_topic);
  planning_utils::loadROSParam(nh_, "topics/body_plan", body_plan_topic);

  planning_utils::loadROSParam(nh_, "/legged_body_planner/goal_state",
                               goal_state_vector);
  planning_utils::loadROSParam(nh, "/legged_body_planner/update_rate",
                               update_rate_);
  planning_utils::loadROSParam(nh, "/legged_body_planner/replan", replan_);

  // // Initialize Trajectories Publisher
  trajectories_publisher_ptr_.reset(
      new TrajectoriesPublisher(nh, topic_prefix));

  // Initialize planner configuration
  planner_config_.loadParams(nh);

  // Setup publisher and subscribers
  planning_algorithm_sub_ =
      nh_.subscribe(plan_topic, 1, &LeggedBodyPlanner::planCallback, this);

  body_plan_pub_ = nh_.advertise<legged_body_msgs::Plan>(body_plan_topic, 1);

  // Set planning parameters
  first_plan_ = true;
  retrieved_plan_ = false;
}

void LeggedBodyPlanner::observerCallback(
    const ocs2_msgs::mpc_observation::ConstPtr& msg) {
  std::lock_guard<std::mutex> lock(latest_observation_mutex_);
  latest_observation_ = ocs2::ros_msg_conversions::readObservationMsg(*msg);
}

void LeggedBodyPlanner::planCallback(
    const legged_body_msgs::Plan::ConstPtr& plan) {
  // Goal: Subscribes to generic plan and converts to rigid body plan
  // Assumptions:
  // - If planner has replan enabled, and if a plan is rececived, then the
  // following happens:
  //    - Assumes that plan received is new & valid
  //    - Clears the original plan
  //    - Use the plan and convert to rigid body plan
  ROS_INFO_THROTTLE(1, "Received plan");

  retrieved_plan_ = false;  // Assumes plan has not been retrieved
  if (first_plan_ || replan_) {
    // Clear plan
    body_plan_.times.clear();
    body_plan_.states.clear();
    body_plan_.controls.clear();

    // Transfers plan data to body plan data
    if (!planToRigidBodyPlan(plan, body_plan_, planner_config_)) {
      // TODO (AZ): Make this more 'productive' later
      ROS_WARN("Plan to rigid body plan transfer not successful... Warning!");
      return;
    }
    ROS_INFO_THROTTLE(1, "Converted to rigid body plan");

    retrieved_plan_ = true;
  }
}

bool LeggedBodyPlanner::planToRigidBodyPlan(
    const legged_body_msgs::Plan::ConstPtr& plan,
    legged_body_msgs::Plan& rigid_body_plan,
    const planning_utils::PlannerConfig& planner_config) {
  // Following planToRigidBodyPlan assumes that
  // 1) plan sends time trajectory w/ observer time information
  // 2) plan gets current state information
  //
  // TODO (AZ): Address if planner should send tiem component portion in future

  // Check if trajectories have same length
  if (!planning_utils::checkTrajectoriesAppropriateSize(plan)) {
    return false;
  }

  const std::vector<double> time_trajectories =
      plan->times;  // Figure out how to make constant w/o triggering
  const std::vector<legged_body_msgs::State> state_trajectories = plan->states;
  const std::vector<legged_body_msgs::Control> control_trajectories =
      plan->controls;

  // Time & State
  // TODO (AZ): Need to see if this is ok here
  std::size_t N = state_trajectories.size();  // Trajectory length
  rigid_body_plan.states.resize(N);
  for (std::size_t i = 0; i < N; i++) {
    rigid_body_plan.times.push_back(time_trajectories[i]);
    planning_utils::stateToRigidBodyState(state_trajectories[i].value,
                                          rigid_body_plan.states[i].value,
                                          planner_config_);
  }

  // Control
  N = control_trajectories.size();
  rigid_body_plan.controls.resize(N);
  for (std::size_t i = 0; i < N; i++) {
    planning_utils::controlToRigidBodyControl(
        control_trajectories[i].value, rigid_body_plan.controls[i].value,
        latest_observation_, planner_config_);
  }
  return true;
}

void LeggedBodyPlanner::publishCurrentPlan() {
  // Conditions for publishing
  // 1) Plan has been retrieved
  // 2) Plan is the first plan OR replan is enabled
  // 3) Planner has not been terminated (TODO(AZ): Needs to be implemented)

  // ROS_INFO_THROTTLE(2, "Retrieved plan %d | Replan: %d", retrieved_plan_,
  //                   replan_);
  ROS_INFO_THROTTLE(2, "Retrieved plan %d", retrieved_plan_);
  if (retrieved_plan_ && (first_plan_ || replan_)) {
    // ROS_INFO_THROTTLE(1, "Publishing Plan");
    body_plan_pub_.publish(body_plan_);
    retrieved_plan_ =
        false;  // Plan has been pub, new plan has not been retrieved
  }
  // After publlishing, if plan was first plan, turn to false
  if (first_plan_) first_plan_ = false;
}

void LeggedBodyPlanner::spin() {
  ros::Rate r(update_rate_);

  while (ros::ok()) {
    // Process callbacks
    ros::spinOnce();

    // Call planner (TODO (AZ): Add plan class)

    // Publish rigid body plan
    publishCurrentPlan();

    // Transform body plan to target trajectories
    trajectories_publisher_ptr_->spinOnce();
    r.sleep();
  }
}
