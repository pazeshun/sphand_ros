cmake_minimum_required(VERSION 2.8.3)
project(sphand_driver)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  baxter_core_msgs
  controller_manager
  dynamixel_controllers
  dynamixel_msgs
  force_proximity_ros
  hardware_interface
  message_generation
  roscpp
  std_msgs
  transmission_interface
)

## https://answers.ros.org/question/247803/mraa-library-inclusion-in-project/
find_package(PkgConfig REQUIRED)
pkg_check_modules(mraa REQUIRED mraa)

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
  baxter_core_msgs
  controller_manager
  dynamixel_controllers
  dynamixel_msgs
  force_proximity_ros
  hardware_interface
  message_runtime
  roscpp
  std_msgs
  transmission_interface
  DEPENDS mraa
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${mraa_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(gripper_v7_loop_node src/gripper_v7_loop.cpp)

## Add cmake target dependencies of the executable
add_dependencies(gripper_v7_loop_node ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(gripper_v7_loop_node ${mraa_LIBRARIES} ${catkin_LIBRARIES})

#############
## Install ##
#############

# XXX

#############
## Testing ##
#############

# XXX

