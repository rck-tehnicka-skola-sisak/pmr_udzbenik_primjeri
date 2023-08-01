cmake_minimum_required(VERSION 3.0.2)
project(stage_simulation)

find_package(catkin REQUIRED COMPONENTS
  rospy
  stage_ros)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS})
