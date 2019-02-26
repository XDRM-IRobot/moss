cmake_minimum_required(VERSION 3.1)
project(moss_recoder)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    image_transport
    roscpp
    sensor_msgs
    std_msgs
    moss_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(CATKIN_DEPENDS 
    cv_bridge
    image_transport
    roscpp
    sensor_msgs
    std_msgs
    moss_msgs
)

include_directories(
    "include"
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(recoder_node
    src/video_recoder_node
)

target_link_libraries(recoder_node
    ${catkin_LIBRARIES}    
    ${OpenCV_LIBS}
)