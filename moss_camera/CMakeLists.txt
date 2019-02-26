cmake_minimum_required(VERSION 3.1)
project(moss_camera)
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(CATKIN_DEPENDS 
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)

include_directories(
  "include"
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(camera_node
  src/camera_node.cpp
  src/uvc_camera.cpp
)

target_link_libraries(camera_node
  ${catkin_LIBRARIES}    
  ${OpenCV_LIBS}
)