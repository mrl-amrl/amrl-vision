cmake_minimum_required(VERSION 2.8.3)
project(amrl_image_view)

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
file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp include/image_view/*.hpp include/image_view/*.h)
include_directories(
include
  ${catkin_INCLUDE_DIRS}
)
add_library(
  image_view
include/image_view/image_view.h
 src/image_view.cpp)

add_executable(image_view_node src/node.cpp)
target_link_libraries(image_view ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(image_view_node image_view)

include_directories(${OpenCV_INCLUDE_DIRS})

add_library(image_view_nodelet src/nodelet.cpp)
target_link_libraries(image_view_nodelet image_view)
