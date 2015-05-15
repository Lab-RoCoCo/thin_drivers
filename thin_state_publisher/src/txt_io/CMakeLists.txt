SET(TXT_IO_LIBRARY_SRCS 
  property.h property.cpp
  base_message.h base_message.cpp
  static_transform_message.cpp 
  base_sensor_message.h base_sensor_message.cpp
  base_image_message.h base_image_message.cpp
  pinhole_image_message.h pinhole_image_message.cpp
  message_factory.cpp message_factory.h
  message_reader.cpp message_reader.h
  message_writer.cpp message_writer.h
  sensor_message_sorter.h sensor_message_sorter.cpp
  static_transform_tree.h static_transform_tree.cpp
  tf_overrider_trigger.h tf_overrider_trigger.cpp;
  message_dumper_trigger.h message_dumper_trigger.cpp
  message_enlister_trigger.h message_enlister_trigger.cpp
)

ADD_LIBRARY(thin_txt_io_library SHARED
  ${TXT_IO_LIBRARY_SRCS}
)

#add_definitions(-DEIGEN_DONT_PARALLELIZE)	     
#add_definitions(-D_GO_PARALLEL_)	     

TARGET_LINK_LIBRARIES(thin_txt_io_library
  thin_globals_library
  ${OpenCV_LIBS} 
  ${OPENGL_glu_LIBRARY}
  ${OPENGL_gl_LIBRARY} 
  ${Boost_FILESYSTEM_LIBRARY} 
  ${Boost_SYSTEM_LIBRARY}
)

ADD_EXECUTABLE(thin_txt_io_dump_reader_app txt_io_dump_reader_app.cpp)

TARGET_LINK_LIBRARIES(thin_txt_io_dump_reader_app
  thin_globals_library
  thin_txt_io_library
  ${OpenCV_LIBS} 
  ${OPENGL_glu_LIBRARY}
  ${OPENGL_gl_LIBRARY} 
  ${Boost_FILESYSTEM_LIBRARY} 
  ${Boost_SYSTEM_LIBRARY}
)

