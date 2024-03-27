# Legged Body Planner

## Overview
This package implements a multi command interface architecture, where given a planning algorithm, converts the plan to a quadruped motion plan. 
Given a planner that publishes trajectories into the `plan` topic, the legged body planner converts the plan into a quadruped plan. This is then
published to the ROS trajectories manager from OCS2.

### License
This source code is released under a [MIT License](legged_planner/LICENSE).

**Author : Andrew Zheng, Sriram Krishnamoorthy <br />
Affiliation : Distributed Intelligence and Robot Autonomy Lab <br />
Maintainers : Andrew Zheng (azheng@clemson.edu) and Sriram Krishnamorthy (sriramk@clemson.edu)**


### Publication


### Unit Tests
Run the unit tests with
```
roslaunch legged_body_planner load_global_params.launch
catkin test -i legged_body_planner
```

## Usage
Run the main node with
```
roslaunch legged_body_planner legged_body_plan.launch
``` 

  
## Config Files
* **legged_body_planner.yaml** Sets planner and planning algorithm hyperparameters


## Nodes

### legged_body_planner
Publishes body plan to a MPC target trajectories. The node communicates with a planning topic of user choice as seen in the node source files (e.g. `src/MultiCommandInterfaceRosNode.cpp`) and transforms the plan to a quadruped plan. There are various options for the different command interface that a user wishes to utilize:
- Robot Velocity: Useful for giving velocity commands only
- Robot Goal: Useful for giving a command for the robot to go towards a specific target point
- Robot Plan: Useful for giving a trajectory of time, states, and control for the robot to follow

Correspondingly, if these interfaces are not the command interface that a user would like to use, users can make use of the **Robot Command** abstract class to implement their own command interface.

## Bugs & Feature Requests
Please report bugs and request features using the [Issue Tracker](https://github.com/AndrewZheng-1011/legged_planner/issues)

