cmake_minimum_required(VERSION 2.8.3)
project(amrl_vision_common)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  sensor_msgs
  std_msgs
  geometry_msgs
)


add_message_files(
  FILES
  Box.msg
  Point.msg
  Boxes.msg
  Perception.msg
  Perceptions.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SetEnabled.srv
)


generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
  geometry_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES amrl-vision-common
 CATKIN_DEPENDS message_generation sensor_msgs std_msgs geometry_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

