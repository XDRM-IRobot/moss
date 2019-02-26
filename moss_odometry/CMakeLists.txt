cmake_minimum_required(VERSION 3.1)
project(moss_odometry)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  tf
  std_msgs
  moss_msgs
)

catkin_package(CATKIN_DEPENDS 
  roscpp
  geometry_msgs
  tf
  std_msgs
  moss_msgs 
)

include_directories(
  "include"
  ${catkin_INCLUDE_DIRS}
)

add_executable(vel2odom_node src/vel2odom.cpp src/vel2odom_node.cpp)
target_link_libraries(vel2odom_node ${catkin_LIBRARIES})

add_executable(tf_node src/tf_tree.cpp

)

target_link_libraries(tf_node ${catkin_LIBRARIES})


add_dependencies(vel2odom_node 
  moss_msgs_generate_messages)


add_dependencies(tf_node 
  moss_msgs_generate_messages)

