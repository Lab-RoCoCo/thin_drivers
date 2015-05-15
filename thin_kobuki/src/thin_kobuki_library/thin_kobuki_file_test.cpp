#include "serial.h"
#include "kobuki_robot.h"
#include "kobuki_protocol.h"
#include <iostream>
#include <fstream>
#include <linux/joystick.h>

using namespace std;


int main (int argc, char** argv) {


  KobukiRobot r;
  ifstream is(argv[1]);
  r.runFromFile(is);
}
