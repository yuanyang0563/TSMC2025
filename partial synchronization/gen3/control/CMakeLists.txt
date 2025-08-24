cmake_minimum_required(VERSION 3.10)
project(control)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  message_generation
)

add_message_files(
   FILES
   Twist.msg
   TwistCommand.msg
)

generate_messages(
   DEPENDENCIES
   geometry_msgs
   sensor_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES control
  CATKIN_DEPENDS message_runtime geometry_msgs roscpp rospy sensor_msgs
  DEPENDS system_lib
)


include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(control src/control.cpp src/common.cpp src/manipulator.cpp)
target_link_libraries(control ${catkin_LIBRARIES})
