#include "carmen_get_time.h"
#include <sys/time.h>

double carmen_get_time(){
  struct timeval tv;
  gettimeofday(&tv, 0);
  return tv.tv_sec + 1e-6*tv.tv_usec;
}
