# How to launch

This package makes use of the OCS2 repository to translate any general planning algorithms (e.g. single integrator, double integrator, rigid body model, etc.)  into a quadrupedal planning algorithm.
This package differs from others in the sense that it allows high fidelity planning for quadrupeds and not simply velocity control. Moreso, it is very intuitive to integrate personal motion planning algorithms into this framework. You simply need to publish to topic `plan`

## Run package:
Initialize simulation
```
roslaunch ocs2_legged_robot_ros legged_robot_sqp.launch
roslaunch legged_body_planner legged_body_plan.launch
```

Run planner in script files
- Ex:
```
rosrun legged_body_planner pub_body_plan_demo.py
```

