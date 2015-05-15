add_library(thin_pioneer_library
  pioneer_robot.cpp
  carmen_get_time.c  
  carmenserial.c  
  pioneer_lib.c)

add_executable(thin_pioneer_test
  thin_pioneer_test.cpp)

target_link_libraries(thin_pioneer_test
  thin_pioneer_library)
