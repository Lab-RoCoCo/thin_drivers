#include "capybara_robot.h"
using namespace std;

CapybaraRobot robot;

int main(int argc, char** argv) {
  robot.connect("/dev/ttyUSB0");
  while (1) {					
    robot.spinOnce();
  }
}
