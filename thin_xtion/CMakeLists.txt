cmake_minimum_required(VERSION 2.8.3)
project(thin_xtion)

# additional cmake modules (OpenNI2)
LIST(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)

# Find OpenNI2
find_package(OpenNI2 REQUIRED)
if(OpenNI2_FOUND)
    message("-- OpenNI2 was found")
    include_directories(${OpenNI2_INCLUDE_DIRS})
else(OpenNI2_FOUND)
    message(WARNING "OpenNI2 was not found, please install OpenNI2 (for example ros-distro-freenect-camera package) if you want to compile thin_xtion")
    return()
endif(OpenNI2_FOUND)

# ROS related stuff
SET(ROS_BUILD_TYPE Release)
SET(CMAKE_C_FLAGS_RELEASE "-O3")
SET(CMAKE_CXX_FLAGS "-O3")
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
)

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
   ${OpenNI2_LIBRARY}
 )

