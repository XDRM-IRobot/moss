cmake_minimum_required(VERSION 3.1)
project(moss_detect)
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    image_transport
    roscpp
    std_msgs
    geometry_msgs
    moss_msgs
)

find_package(OpenCV REQUIRED)


catkin_package( CATKIN_DEPENDS
    cv_bridge
    image_transport
    roscpp
    std_msgs
    geometry_msgs
    moss_msgs
)

include_directories(
  "include"
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(armor_detect_node  
    src/armor_info.cpp
    src/armor_detect.cpp
    src/angle_solver.cpp
    src/armor_detect_node.cpp
)

target_link_libraries(armor_detect_node 
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)

add_dependencies(armor_detect_node 
  moss_msgs_generate_messages)