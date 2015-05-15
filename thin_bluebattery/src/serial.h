#pragma once
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#ifdef __cplusplus
extern "C" {
#endif
  
  //! returns the descriptor of a serial port
  int serial_open(const char* name);

  //! sets the attributes
  int serial_set_interface_attribs (int fd, int speed, int parity);
  
  //! puts the port in blocking/nonblocking mode
  void serial_set_blocking (int fd, int should_block);

  //! write on the serial port
  int serial_write(int fd, const char* buffer, int len);

  //! reads from the serial port (timeout is in seconds): return -1 means timeout, return 0 means disconnection, otherwise the number of bytes read is returned
  int serial_read(int fd, char* buffer, int len, double timeout);

  //! reads a line from the serial port (timeout is in seconds): return -1 means timeout, return 0 means disconnection, otherwise the number of bytes read is returned
  int serial_readline(int fd, char* buffer, int len, double timeout);

#ifdef __cplusplus
}
#endif



