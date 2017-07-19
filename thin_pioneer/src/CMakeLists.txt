add_subdirectory(thin_pioneer_library)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(thin_pioneer_node
  thin_pioneer_node.cpp
)

target_link_libraries(thin_pioneer_node
  thin_pioneer_library
  ${catkin_LIBRARIES}
)
