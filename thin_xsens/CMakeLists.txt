cmake_minimum_required(VERSION 2.8.3)
project(thin_xsens)

find_path(_mtsdk_found mtsdk.so PATHS ./mtsdk)
message(STATUS ${_mtsdk})
if(NOT _mtsdk_found)
    message(WARNING "No mtsdk found in . directory, please download and install it if you want to compile thin_xsens")
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

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ./mtsdk/include
  ./mtsdk/xcommunication/include
  ./mtsdk/xstypes/include
)

link_directories(
  ./mtsdk
  ./mtsdk/xcommunication
  ./mtsdk/xstypes)

add_executable(thin_xsens_node src/thin_xsens_node.cpp)
target_link_libraries(thin_xsens_node
                      ${catkin_LIBRARIES} 
                      mtsdk 
                      xcommunication 
                      xstypes 
                      pthread 
                      rt 
                      dl )

