cmake_minimum_required(VERSION 2.8.3)
project(sphand_driver)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  controller_manager
  hardware_interface
  message_generation
  roscpp
)

catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

add_action_files(
  FILES
  CalibJoint.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

# XXX

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
  actionlib
  actionlib_msgs
  controller_manager
  hardware_interface
  message_runtime
  roscpp
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(gripper_v7_realtime_loop_node src/gripper_v7_realtime_loop.cpp)

## Add cmake target dependencies of the executable
add_dependencies(gripper_v7_realtime_loop_node ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(gripper_v7_realtime_loop_node mraa ${catkin_LIBRARIES})

#############
## Install ##
#############

# XXX

#############
## Testing ##
#############

# XXX

