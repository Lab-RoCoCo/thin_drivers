#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <math.h>
#include "serial.h"

int serial_set_interface_attribs (int fd, int speed, int parity) {
  struct termios tty;
  memset (&tty, 0, sizeof tty);
  if (tcgetattr (fd, &tty) != 0) {
    printf ("error %d from tcgetattr", errno);
    return -1;
  }

  cfsetospeed (&tty, speed);
  cfsetispeed (&tty, speed);
  cfmakeraw(&tty);
  // enable reading
  tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
  tty.c_cflag |= parity;

  if (tcsetattr (fd, TCSANOW, &tty) != 0) {
      printf ("error %d from tcsetattr", errno);
      return -1;
    }
  return 0;
}

void serial_set_blocking (int fd, int should_block) {
  struct termios tty;
  memset (&tty, 0, sizeof tty);
  if (tcgetattr (fd, &tty) != 0)
    {
      printf ("error %d from tggetattr", errno);
      return;
    }

  tty.c_cc[VMIN]  = should_block ? 1 : 0;
  tty.c_cc[VTIME] = 5;            // 0.5 timeout read timeout

  if (tcsetattr (fd, TCSANOW, &tty) != 0)
    printf ("error %d setting term attributes", errno);
}

int serial_open(const char* name) {
  int fd = open (name, O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0) {
    printf ("error %d setting term attributes", errno);
  }
  return fd;
}

int serial_write(int fd, const char* buffer, int len) {
  write(fd, buffer, len);
}

int serial_read(int fd, char* buffer, int len, double timeout_sec) {
  int try_read = (timeout_sec < 0.0 ? 1 : 0);
  if (timeout_sec > 0.0) {
    fd_set read_fds, write_fds, except_fds;
    FD_ZERO(&read_fds); FD_ZERO(&write_fds); FD_ZERO(&except_fds);
    FD_SET(fd, &read_fds);
    struct timeval timeout;
    timeout.tv_sec = floor(timeout_sec);
    timeout.tv_usec = (timeout_sec - timeout.tv_sec) * 1e6;
    if (select(fd + 1, &read_fds, &write_fds, &except_fds, &timeout) == 1) try_read = 1;
  }
  if (try_read) {
    int r = read(fd, buffer, len);
    buffer[r] = '\0';
    return r;
  }
  else return -1;   // timeout
}

int serial_readline(int fd, char* buffer, int len, double timeout) {
  char* curs = buffer;
  while (curs == buffer || *(curs-1) /*buffer[curs-buffer-1]*/ != '\n') {
    int r = serial_read(fd, curs, 1, timeout);
    if (r == 0) return 0;   // disconnection?
    else if (r == -1) return -1;    // timeout
    else curs += r;
  }
  return curs - buffer;
}

