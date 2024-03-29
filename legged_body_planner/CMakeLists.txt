cmake_minimum_required(VERSION 3.0.2)
project(legged_body_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries

find_package(catkin REQUIRED COMPONENTS
  angles
  std_msgs
  geometry_msgs
  ocs2_msgs
  legged_body_msgs
  legged_body_utils
  roscpp
  rospy
  tf2
  ocs2_legged_robot_ros
  ocs2_robotic_tools
  ocs2_ros_interfaces
  grid_map_core
  grid_map_ros
)


find_package(Boost REQUIRED COMPONENTS 
  system
  filesystem
  )



################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS angles geometry_msgs roscpp tf2 ocs2_robotic_tools ocs2_ros_interfaces std_msgs legged_body_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(robot_command_LIB_SOURCES
  src/robot_command/RobotCommand.cpp
  src/robot_command/RobotVelocityCommand.cpp
  src/robot_command/RobotGoalCommand.cpp
  src/robot_command/RobotPlanCommand.cpp
)

set(motion_adapters_LIB_SOURCES
  src/motion_adapters/LeggedRobotAdapter.cpp
  )

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/MultiCommandInterfaceRos.cpp
  ${robot_command_LIB_SOURCES}
  ${motion_adapters_LIB_SOURCES}
)

## Add cmake target dependencies of the library
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(multi_command_interface_ros_node src/MultiCommandInterfaceRosNode.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(multi_command_interface_ros_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) 

## Specify libraries to link a library or executable target against
target_link_libraries(multi_command_interface_ros_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(robot_command-test
   test/test_robot_command.cpp)
if(TARGET robot_command-test)
   target_link_libraries(robot_command-test ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()

catkin_add_gtest(motion_adapter-test
   test/test_motion_adapter.cpp)
if(TARGET motion_adapter-test)
   target_link_libraries(motion_adapter-test ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
    add_clang_tooling(
            TARGETS ${PROJECT_NAME} legged_target_trajectories_publisher
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            CF_WERROR
    )
endif (cmake_clang_tools_FOUND)
