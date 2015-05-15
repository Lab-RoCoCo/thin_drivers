project( xstypes )

cmake_minimum_required (VERSION 2.8) 
cmake_policy(SET CMP0015 NEW)

set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -o3 -pthread")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}  -o3 -pthread")

include_directories( ./include/xsens )

aux_source_directory( src XSTYPES_SRC )

add_library(xstypes STATIC ${XSTYPES_SRC})
set_target_properties(xstypes PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})
