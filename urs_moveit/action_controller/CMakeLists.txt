cmake_minimum_required(VERSION 2.8.3)
project(action_controller)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(Boost REQUIRED thread)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  pluginlib
  actionlib
  roscpp
  trajectory_msgs
  actionlib_msgs
  std_msgs
  message_generation
)

add_message_files(
  FILES
  MultiDofFollowJointTrajectoryFeedback.msg
  MultiDofFollowJointTrajectoryGoal.msg
  MultiDofFollowJointTrajectoryResult.msg
  MultiDofFollowJointTrajectoryActionFeedback.msg
  MultiDofFollowJointTrajectoryActionGoal.msg
  MultiDofFollowJointTrajectoryActionResult.msg
  MultiDofFollowJointTrajectoryAction.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  trajectory_msgs
  actionlib_msgs
  action_controller
)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  DEPENDS
  CATKIN_DEPENDS message_runtime
  moveit_core
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# add_library(action_controller src/actionController.cpp)
add_executable(action_controller src/actionController.cpp)
target_link_libraries(action_controller ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

#install(TARGETS action_controller LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
