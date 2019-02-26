cmake_minimum_required(VERSION 3.1)
project(moss_serial)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    moss_msgs
)

include_directories(
  "include"
  ${catkin_INCLUDE_DIRS}
)

catkin_package(CATKIN_DEPENDS 
    roscpp
    std_msgs
    geometry_msgs
    moss_msgs 
)

add_executable(serial_read_node  
    src/serial_read_node.cpp)

target_link_libraries(serial_read_node 
    ${catkin_LIBRARIES})

add_executable(serial_write_node 
    src/serial_write_node.cpp)

target_link_libraries(serial_write_node 
    ${catkin_LIBRARIES})

add_dependencies(serial_read_node 
  moss_msgs_generate_messages)

add_dependencies(serial_write_node 
  moss_msgs_generate_messages)