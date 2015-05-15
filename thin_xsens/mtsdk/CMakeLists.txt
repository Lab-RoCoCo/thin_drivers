project( mtsdk )

cmake_minimum_required (VERSION 2.8) 
cmake_policy(SET CMP0015 NEW)

set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -o3 -pthread")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}  -o3 -pthread")

add_subdirectory(xstypes)
add_subdirectory(xcommunication)

include_directories( ./include
                     ./xcommunication/include
                     ./xstypes/include)

link_directories(xstypes
                 xcommunication)

aux_source_directory( src MTSDK_SRC )

add_library(mtsdk STATIC ${MTSDK_SRC})
set_target_properties(mtsdk PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})

add_executable(example_orientation example_orientation.cpp)
target_link_libraries( example_orientation mtsdk xcommunication xstypes pthread rt dl )
set_target_properties(example_orientation PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_executable(example_calib example_calib.cpp)
target_link_libraries( example_calib mtsdk xcommunication xstypes pthread rt dl )
set_target_properties(example_calib PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)