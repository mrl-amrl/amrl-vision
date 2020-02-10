cmake_minimum_required(VERSION 2.8.3)
project(vision_launch)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  OpenCV
  cv_bridge
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
