cmake_minimum_required(VERSION 2.8.3)
project(amrl_ipcamera)


find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge dynamic_reconfigure image_transport sensor_msgs)
find_package( OpenCV REQUIRED )
include_directories(include ${catkin_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS} )
catkin_package(
 INCLUDE_DIRS
  include
 CATKIN_DEPENDS
  roscpp cv_bridge dynamic_reconfigure image_transport sensor_msgs
 DEPENDS
  OpenCV
)
add_executable(ipcamera_node src/ipcamera_node.cpp include/ipcamera/ipcamera_node.hpp)
target_link_libraries(ipcamera_node ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
