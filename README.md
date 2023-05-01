# legged_planner
<p align="center">
  <img src="docs/anybotics_legged_planner.gif" alt="animated" />
</p>

## Overview
The package `legged_planner` makes use of the OCS2 repository to translate any general planning algorithms (e.g. single integrator, double integrator, rigid body model, etc.)  into a quadrupedal planning algorithm.
This package differs from others in the sense that it allows for high fidelity planning for quadrupeds (and not simply velocity control) with extreme ease.

## Dependencies
This framework relies on the OCS2 repository, so the following dependencies needs to be installed. **NOTE**: OCS2 is a large repository, and so this repo will only build packages pertaining to legged robots. This package assumes the user is using Linux Distro Ubuntu 20.04 (Focal Fossa) and has set up the corresponding ros-distro (ros noetic) with a catkin workspace.

### Clone source code ###
```
# clone the repository
git clone git@github.com:AndrewZheng-1011/legged_planner.git
```

### Clone OCS2 ###
```
git clone git@github.com:leggedrobotics/ocs2.git
git clone --recurse-submodules https://github.com/leggedrobotics/pinocchio.git
git clone --recurse-submodules https://github.com/leggedrobotics/hpp-fcl.git
git clone https://github.com/leggedrobotics/ocs2_robotic_assets.git
```

### Install Other Dependencies ###
```
sudo apt install liburdfdom-dev liboctomap-dev libassimp-dev
# Python dependencies
pip3 install sympy # Symbolic python package
pip3 install numpy # Numpy
# ROS Dependencies for catkin tools
sudo apt-get install ros-$ROS_DISTRO-catkin python3-catkin-tools -y
sudo apt-get install ros-$ROS_DISTRO-grid-map
# Eigen library
sudo apt-get install libeigen3-dev
# Doxygen
sudo apt-get install doxygen doxygen-doc doxygen-gui graphviz
```

### Setup workspace & Build ###
Configure the workspace with debug info
```
catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo
catkin config --extend /opt/ros/noetic # Extend only to default ros
```

Build ocs2 packages
```
catkin build ocs2_legged_robot_ros ocs2_self_collision_visualization
```

Build the source code of `legged_body_planner`
```
catkin build legged_body_planner legged_body_msgs
```


## Run Package:
Initialize simulation
```
roslaunch ocs2_legged_robot_ros legged_robot_sqp.launch
# Run one of the 'walking' gait sequences before running next launch file
roslaunch legged_body_planner legged_body_plan.launch
```


## Personal Algorithms
It is very intuitive to run this planning framework with a generic planning algorithms. Simply publish a plan in the form of a `Plan` msg, and the legged body planner node will handle the rest. See demo files in the scripts directory for an example

