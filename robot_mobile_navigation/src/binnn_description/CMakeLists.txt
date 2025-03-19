cmake_minimum_required(VERSION 2.8.3)
project(binnn_description)

find_package(catkin REQUIRED COMPONENTS
  joint_state_publisher
  robot_state_publisher
  urdf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
