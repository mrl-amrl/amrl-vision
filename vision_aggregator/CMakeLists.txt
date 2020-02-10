cmake_minimum_required(VERSION 2.8.3)
project(amrl_vision_aggregator)

add_compile_options(-std=c++11)

find_package(OpenCV 3.4.9 EXACT REQUIRED PATHS /usr/local/include/opencv2)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  image_transport
  roscpp
  rospy
  std_msgs
  amrl_vision_common
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS message_runtime
 cv_bridge 
 image_transport
 roscpp 
 rospy 
 std_msgs
 DEPENDS OpenCV
)
file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp include/vision_aggregator/*.hpp include/vision_aggregator/*.h)
include_directories(
include
  ${catkin_INCLUDE_DIRS}
)
add_library(
  vision_aggregator
include/vision_aggregator/vision_aggregator.h
 src/vision_aggregator.cpp)

add_executable(vision_aggregator_node src/node.cpp)
target_link_libraries(vision_aggregator ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(vision_aggregator_node vision_aggregator)

include_directories(${OpenCV_INCLUDE_DIRS})

# add_dependencies(vision_aggregator ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
# target_link_libraries(vision_aggregator ${catkin_LIBRARIES} ${OpenCV_LIBS})
