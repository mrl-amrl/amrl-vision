cmake_minimum_required(VERSION 2.8.3)
project(amrl_motion_detection)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  image_transport
  message_generation
  roscpp
  rospy
  std_msgs
  nodelet
  amrl_vision_common
)

# add_service_files(
#   FILES
#   SetEnabled.srv
# )

generate_messages(
  DEPENDENCIES
  std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/MotionDetection.cfg
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS message_runtime
 cv_bridge 
 dynamic_reconfigure 
 image_transport 
 message_generation 
 roscpp 
 rospy 
 std_msgs
 nodelet
 DEPENDS OpenCV
)
file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp include/motion_detection/*.hpp include/motion_detection/*.h)
include_directories(
include
  ${catkin_INCLUDE_DIRS}
)
add_library(
  motion_detection
include/motion-detection/motion_detection.h
 src/motion_detection.cpp)

add_executable(motion_detection_node src/node.cpp)
target_link_libraries(motion_detection_node ${catkin_LIBRARIES})
target_link_libraries(motion_detection_node motion_detection)

include_directories(${OpenCV_INCLUDE_DIRS})

add_library(motion_detection_nodelet src/nodelet.cpp)
target_link_libraries(motion_detection_nodelet motion_detection)

# Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/motion-detection.cpp
# )
add_dependencies(motion_detection ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(motion_detection ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(motion_detection ${PROJECT_NAME}_gencfg)
add_dependencies(motion_detection_node ${PROJECT_NAME}_gencfg)
