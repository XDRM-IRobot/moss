cmake_minimum_required(VERSION 3.1)
project(moss_decision)
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  
  roscpp
  tf
  std_msgs
  move_base_msgs
  geometry_msgs
  moss_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(CATKIN_DEPENDS 
  actionlib
  roscpp
  tf
  moss_msgs
)

include_directories(
  "include"
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(decision_node src/decision_node.cpp)

target_link_libraries(decision_node
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBRARIES}
)

add_dependencies(decision_node
  moss_msgs_generate_messages)