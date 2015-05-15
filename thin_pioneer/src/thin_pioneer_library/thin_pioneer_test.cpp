#include "pioneer_robot.h"
#include <iostream>
#include <linux/joystick.h>
#include <cstdio>
 #include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

using namespace std;


int main (int argc, char** argv) {

  int fd = open ("/dev/input/js0", O_RDONLY | O_NONBLOCK);
  if (fd<0) {
    cerr << "no joy found" << endl;
  }
  double tv=0;
  double rv=0;
  double tvscale = -0.5/32767;
  double rvscale = -1./32767;
  PioneerRobot r;
  r.connect("p3at", "/dev/ttyUSB0");
  while (1) {
    struct js_event e;
    while (read (fd, &e, sizeof(e)) > 0 ){
      printf("%u %d %d %d\n", e.time, e.value, e.type, e.number);
      int axis = e.number;
      int value = e.value;
      if (axis == 1) {
      	tv = value * tvscale;
      }
      if (axis == 3) {
      	rv = value*rvscale;
      }
    }
    r.setSpeed(tv,rv);
    r.spinOnce();
    double x,y,theta;
    r.getOdometry(x,y,theta);
    r.setSpeed(tv, rv);
    cerr << "speed: [ " << tv << " " << rv << " ]";
    cerr << " pose:  [ " << x << " " << y << " " << theta << " ]" << endl;
  }
}
