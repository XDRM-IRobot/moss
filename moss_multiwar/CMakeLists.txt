cmake_minimum_required(VERSION 3.1)
project(moss_multiwar)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    std_msgs
    moss_msgs
)

catkin_package(CATKIN_DEPENDS 
    roscpp 
    sensor_msgs 
    std_msgs
    moss_msgs 
)

include_directories(
    "include"
    ${catkin_INCLUDE_DIRS}
)

add_executable(udp_server src/server_node.cpp)
add_executable(udp_client src/client_node.cpp)

target_link_libraries(udp_server ${catkin_LIBRARIES} )
target_link_libraries(udp_client ${catkin_LIBRARIES} )

add_dependencies(udp_server 
  moss_msgs_generate_messages)

add_dependencies(udp_client
  moss_msgs_generate_messages)