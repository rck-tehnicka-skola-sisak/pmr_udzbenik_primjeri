cmake_minimum_required(VERSION 3.0.2)
project(bug_navigation)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  sensor_msgs
  message_generation
)

add_service_files(
  FILES
  SetBugGoal.srv
)

generate_messages(DEPENDENCIES
  geometry_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
