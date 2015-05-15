cmake_minimum_required(VERSION 2.8.3)
project(thin_bluebattery)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES capybara_robot
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp
#  DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(thin_bluebattery_node src/thin_bluebattery_node.cpp src/serial.c)
target_link_libraries(thin_bluebattery_node ${catkin_LIBRARIES})
