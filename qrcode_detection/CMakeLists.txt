cmake_minimum_required(VERSION 2.8.3)
project(amrl_qrcode_detection)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  tf
  dynamic_reconfigure
  message_generation
  amrl_vision_common
  nodelet
)

add_service_files(
  FILES
  SetEnabled.srv
)

generate_dynamic_reconfigure_options(cfg/QRCode.cfg)

add_message_files(
  FILES
  QRCode.msg
  QRCodes.msg
)

generate_messages(
  DEPENDENCIES
  amrl_vision_common
  std_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
      image_transport
      cv_bridge
      nodelet
    DEPENDS
      OpenCV
)

find_package(ImageMagick COMPONENTS Magick++ REQUIRED)

include_directories(include) 
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${ImageMagick_INCLUDE_DIRS})

add_library(visualizer src/visualizer.cpp)
add_library(qrcode_reader src/qrcode_reader.cpp)
add_library(object_tf src/object_tf.cpp)
add_library(qrcode_detection src/qrcode_detection.cpp)
add_executable(qrcode_reader_node src/node.cpp)

target_link_libraries(qrcode_detection ${catkin_LIBRARIES} qrcode_reader visualizer object_tf ${ImageMagick_LIBRARIES} -lzbar)
target_link_libraries(qrcode_reader_node qrcode_detection)

add_library(qrcode_detection_nodelet src/nodelet.cpp)
target_link_libraries(qrcode_detection_nodelet qrcode_detection)

add_dependencies(qrcode_reader_node ${PROJECT_NAME}_gencfg)
