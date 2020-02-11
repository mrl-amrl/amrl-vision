cmake_minimum_required(VERSION 2.8.3)
project(amrl_mjpeg_streamer)

add_compile_options(-std=c++11)

find_package(OpenCV 3.4.9 EXACT REQUIRED PATHS /usr/local/include/opencv2)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  nodelet
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS
 cv_bridge 
 image_transport
 roscpp 
 nodelet
 DEPENDS OpenCV
)
file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp include/mjpeg_streamer/*.hpp include/mjpeg_streamer/*.h)
include_directories(
include
  ${catkin_INCLUDE_DIRS}
)
add_library(
  mjpeg_streamer
include/mjpeg_streamer/mjpeg_streamer.h
 src/mjpeg_streamer.cpp)

add_library(
  mjpeg_writer
include/mjpeg_streamer/mjpeg_writer.h
 src/mjpeg_writer.cpp)

add_executable(mjpeg_streamer_node src/node.cpp)
target_link_libraries(mjpeg_streamer ${catkin_LIBRARIES} ${OpenCV_LIBS} mjpeg_writer)
target_link_libraries(mjpeg_streamer_node mjpeg_streamer)

include_directories(${OpenCV_INCLUDE_DIRS})

add_library(mjpeg_streamer_nodelet src/nodelet.cpp)
target_link_libraries(mjpeg_streamer_nodelet mjpeg_streamer)
