cmake_minimum_required(VERSION 2.8.3)
project(moveit_simple_controller_manager)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin COMPONENTS
  moveit_core
  pluginlib
  actionlib
  roscpp
  control_msgs
  action_controller
  REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS include
  CATKIN_DEPENDS moveit_core actionlib control_msgs
)

add_library(${PROJECT_NAME} src/moveit_simple_controller_manager.cpp)
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES moveit_simple_controller_manager_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
