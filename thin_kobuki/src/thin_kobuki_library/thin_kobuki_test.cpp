#include "serial.h"
#include "kobuki_robot.h"
#include "kobuki_protocol.h"
#include <iostream>
#include <linux/joystick.h>

using namespace std;


int main (int argc, char** argv) {

  Packet p;
  BaseControlPayload* bpc = new BaseControlPayload;
  bpc->speed = 100;
  bpc->radius =  4;
  p._payloads.push_back(bpc);
  unsigned char buf[1024];
  int k = p.write(buf);
  
  int fd = open ("/dev/input/js0", O_RDONLY|O_NONBLOCK);
  if (fd<0) {
    cerr << "no joy found" << endl;
  }

  KobukiRobot r;
  r.connect(argv[1]);
  r.playSequence(0);
  float tv = 0;
  float rv = 0;
  float tvscale = .1/32767.0;
  float rvscale = 1/32767.0;

  while (1) {
    struct js_event e;
    while (read (fd, &e, sizeof(e)) > 0 ){
      int axis = e.number;
      int value = e.value;
      if (axis == 1) {
	tv = -value * tvscale;
      }
      if (axis == 4) {
	rv = -value *rvscale;
      }
    }
    r.setSpeed(tv,rv);
    r.spinOnce();
    double x,y,theta;
    r.getOdometry(x,y,theta);
    cerr << "speed: [ " << tv << " " << rv << " ]";
    cerr << " pose:  [ " << x << " " << y << " " << theta << " ]" << endl;
  }
}
