project(moss_msgs)
cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  nav_msgs
  geometry_msgs
  actionlib
  message_generation
)

add_action_files(
  DIRECTORY action
  FILES
  LocalPlanner.action
  GlobalPlanner.action
  ArmorDetection.action
)

add_message_files(
  FILES
  armor_info.msg 
  car_info.msg 
  share_info.msg 
  shoot_info.msg 
)

add_service_files(
  DIRECTORY srv
  FILES
  GimbalMode.srv
  FricWhl.srv
  ShootCmd.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_package()
