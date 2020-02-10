cmake_minimum_required(VERSION 2.8.3)
project(amrl_dark_objects)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/DarkObjects.cfg
)

catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)



