cmake_minimum_required(VERSION 2.8.3)
project(binnn)

find_package(catkin REQUIRED COMPONENTS
  gmapping
  binnn_description
  roscpp
  rospy
  teb_local_planner
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(binnn_controller src/binnn_controller.cpp)
target_link_libraries(binnn_controller ${catkin_LIBRARIES})