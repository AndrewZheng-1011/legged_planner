# Legged Body Msgs

## Overview
This package describes messages for describing robot state, control, and robot plan. This package is dependent on std_msgs

### License
The source code is released under a [MIT License](legged_planner/LICENSE)

**Author: Andrew Zheng <br />
Affiliation: Distributed Intelligence and Robot Autonomy Lab <br />
Maintainers: Andrew Zheng (azheng@clemson.edu)**

The Legged Body Msgs has been tested under [ROS] Noetic 20.04

## Messages (.msg)
- `State.msg` : State defining robot. For quadruped robot states, states are defined as $\[v_x, v_y, v_z, yaw_{rate}, pitch_{rate}, roll_{rate}, x, y, z, yaw, pitch$, and  $roll\]$
- `Control.msg` : Control for robot. For quadruped control, torque control of each joints
- `Plan.msg` : A message to hold a plan. Plan is defined as array of time, state, and control trajectories, and the time of plan (`std_msgs/time`)

## Bug & Feature Requests

Please report bugs and requeset features ussing the [Issue Tracker](https://github.com/AndrewZheng-1011/legged_planner/issues)
