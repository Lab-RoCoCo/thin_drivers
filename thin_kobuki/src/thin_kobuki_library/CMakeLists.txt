add_library(thin_kobuki_library
  kobuki_robot.cpp
  kobuki_protocol.cpp
  serial.c)

add_executable (thin_kobuki_test
  thin_kobuki_test.cpp)

target_link_libraries(
  thin_kobuki_test
  thin_kobuki_library
)

add_executable (thin_kobuki_file_test
  thin_kobuki_file_test.cpp)

target_link_libraries(
  thin_kobuki_file_test
  thin_kobuki_library
)



