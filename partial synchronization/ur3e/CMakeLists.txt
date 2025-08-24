cmake_minimum_required (VERSION 3.10)
project(ur3e_control)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs std_msgs geometry_msgs)
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(control src/control.cpp src/common.cpp src/manipulator.cpp)
target_link_libraries(control ${catkin_LIBRARIES})
