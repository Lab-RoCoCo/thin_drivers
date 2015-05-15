cmake_minimum_required(VERSION 2.8.3)
project(thin_xtion)

find_path(_openni_found OpenNI2 PATHS ~/src)
if(NOT _openni_found)
    message(WARNING "No OpenNI2 directory found in ~/src, please install OpenNI2 in this location if you want to compile thin_xtion")
    return()
endif()

SET(ROS_BUILD_TYPE Release)
SET(CMAKE_C_FLAGS_RELEASE "-O3")
SET(CMAKE_CXX_FLAGS "-O3")
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)

include_directories(~/src/OpenNI2/Include)
include_directories(~/src/OpenNI2/Samples/Common/)
include_directories(~/src/OpenNI2/Include/Linux-Arm)
link_directories(~/src/OpenNI2/Bin/Arm-Release)
link_directories(~/src/OpenNI2/Bin/x64-Release)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES xtionberry
#  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(thin_xtion_node src/thin_xtion_node.cpp)
target_link_libraries(thin_xtion_node
   ${catkin_LIBRARIES}
   rt 
   OpenNI2
 )

