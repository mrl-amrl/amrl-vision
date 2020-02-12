cmake_minimum_required(VERSION 2.8.3)
project(amrl_rtsp_streamer)

add_compile_options(-std=c++11)

find_package(OpenCV 3.4.9 EXACT REQUIRED PATHS /usr/local/include/opencv2)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS
 cv_bridge 
 image_transport
 roscpp 
 DEPENDS OpenCV
)
file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp include/rtsp_streamer/*.hpp include/rtsp_streamer/*.h)
include_directories(
include
  ${catkin_INCLUDE_DIRS}
)
add_library(
  rtsp_streamer
include/rtsp_streamer/rtsp_streamer.h
 src/rtsp_streamer.cpp)

add_library(
  mjpeg_writer
include/rtsp_streamer/mjpeg_writer.h
 src/mjpeg_writer.cpp)

add_executable(rtsp_streamer_node src/node.cpp)
target_link_libraries(rtsp_streamer ${catkin_LIBRARIES} ${OpenCV_LIBS} mjpeg_writer)
target_link_libraries(rtsp_streamer_node rtsp_streamer)

include_directories(${OpenCV_INCLUDE_DIRS})
