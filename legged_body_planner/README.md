# Legged Body Planner

## Overview
This package implements a planning architecture, where given any generic planning algorithm, converts the plan to a quadruped motion plan. 
Given a planner that publishes trajectories into the `plan` topic, the legged body planner converts the plan into a body plan. This is then
published to the ROS trajectories manager from OCS2.

### License
This source ccode is released under a [MIT License](legged_planner/LICENSE).

**Author : Andrew Zheng, Sriram Krishnamoorthy <br />
Affiliation : Distributed Intelligence and Robot Autonomy Lab <br />
Maintainers : Andrew Zheng (azheng@clemson.edu) and Sriram Krishnamorthy (sriramk@clemson.edu)**


### Publication


### Unit Tests
Run the unit tests with

    catkin run_tests legged_body_planner

## Usage
Run the main node with

    roslaunch legged_body_planner legged_body_plan.launch
  
## Config Files
* **legged_body_planner.yaml** Sets planner and planning algorithm hyperparameters
* **legged_body_planner_topics.yaml** Sets name of planner to subscribe to

## Nodes

### legged_body_planner
Publishes body plan to a MPC target trajectories. The node communicates with a planning topic and transforms the plan to a quadruped plan. If allow replan mode, the body plan is accepted and
published under the following conditions:
1. Plan has been retrieved
2. Plan is the first plan or replan is enabled
3. (TODO) Planning algorithm has not terminated

#### Subscribed Topics
* **`topics/plan`** ([legged_body_msgs/Plan])

    Generic plan from planning algorithms (A*, RRT, etc.)
    
* **`topics/body_plan`** ([legged_body_msgs/Plan])

    Body plan for the quadruped
    
#### Parameters
* **`replan`** (double, default : 20)

    The update rate of the planner (in Hz).

## Bugs & Feature Requests
Please report bugs and request features using the [Issue Tracker](https://github.com/AndrewZheng-1011/legged_planner/issues)
