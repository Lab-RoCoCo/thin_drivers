add_subdirectory(thin_pioneer_library)


add_executable(thin_pioneer_node
  thin_pioneer_node.cpp
)

target_link_libraries(thin_pioneer_node
  thin_pioneer_library
  ${catkin_LIBRARIES}
)