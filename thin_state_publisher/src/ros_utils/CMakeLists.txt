add_executable(thin_state_publisher_node  thin_state_publisher_node.cpp)
TARGET_LINK_LIBRARIES(thin_state_publisher_node
  thin_ros_wrappers_library
  thin_txt_io_library
  ${catkin_LIBRARIES}
  )


add_executable(thin_message_dumper_node 
  thin_message_dumper_node.cpp 
  )
TARGET_LINK_LIBRARIES(thin_message_dumper_node 
  thin_ros_wrappers_library
  thin_txt_io_library
  ${OpenCV_LIBS}
  ${ZLIB_LIBRARIES}   
  ${catkin_LIBRARIES}
  )


