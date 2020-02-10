cmake_minimum_required(VERSION 2.8.3)
project(amrl_hazmat_detection)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(OpenCV 3.4.9 EXACT REQUIRED PATHS /usr/local/include/opencv2)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  image_transport
  roscpp
  rospy
  std_msgs
  nodelet
  amrl_vision_common
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS message_runtime
 cv_bridge 
 dynamic_reconfigure 
 image_transport 
 roscpp 
 rospy 
 std_msgs
 nodelet
 DEPENDS OpenCV
)
file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp include/hazmat_detection/*.hpp include/hazmat_detection/*.h)
include_directories(
include
  ${catkin_INCLUDE_DIRS}
)
add_library(
  hazmat_detection
include/hazmat_detection/hazmat_detection.h
  src/hazmat_detection.cpp
)

add_library(
  neural_network
include/hazmat_detection/neural_network.h
  src/neural_network.cpp
)

add_executable(hazmat_detection_node src/node.cpp)
target_link_libraries(hazmat_detection_node ${catkin_LIBRARIES})
target_link_libraries(hazmat_detection_node hazmat_detection)

include_directories(${OpenCV_INCLUDE_DIRS})

add_library(hazmat_detection_nodelet src/nodelet.cpp)
target_link_libraries(hazmat_detection_nodelet hazmat_detection)

add_dependencies(hazmat_detection ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(hazmat_detection ${catkin_LIBRARIES} ${OpenCV_LIBS} neural_network)
