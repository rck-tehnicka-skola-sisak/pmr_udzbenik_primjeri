cmake_minimum_required(VERSION 3.0.2)
project(diff_description)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  sensor_msgs
  stage_ros
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
