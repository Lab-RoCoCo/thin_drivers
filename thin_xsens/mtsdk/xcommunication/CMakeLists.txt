project( xcommunication )

cmake_minimum_required (VERSION 2.8) 
cmake_policy(SET CMP0015 NEW)

set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -o3 -pthread")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}  -o3 -pthread")

include_directories( ./include ../xstypes/include )

aux_source_directory( src XCOMMUNICATION_SRC )

add_library(xcommunication STATIC ${XCOMMUNICATION_SRC})
set_target_properties(xcommunication PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})
