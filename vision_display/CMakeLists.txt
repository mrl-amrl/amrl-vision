cmake_minimum_required(VERSION 2.8.3)
project(amrl_vision_display)

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  cv_bridge
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
