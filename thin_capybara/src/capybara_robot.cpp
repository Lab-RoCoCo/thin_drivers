#include "capybara_robot.h"

#include "serial.h"
#include <cstdio>
 #include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdexcept>
#include <cmath>
#include <iostream>
using namespace std;

CapybaraRobot::CapybaraRobot() {
  _x=0;
  _y=0;
  _theta=0;
  _serial_fd = -1;
  _des_rv = 0;
  _des_tv = 0;
  _state=Unsync;
  _baseline = 0.3; // distance between the wheels
  _right_meters_per_tick = 0.001; // dummy values for conversion encoder ticks/distance
  _left_meters_per_tick = 0.001;
  _interval = 1.0/50.0; 
}

void CapybaraRobot::connect(const std::string& device) {
  if (_serial_fd>-1){
    disconnect();
  }

  _serial_fd= serial_open("/dev/ttyUSB0");
  if (_serial_fd<0)
    throw std::runtime_error("unable to open serial port");
  if (serial_set_interface_attribs (_serial_fd, B115200, 0))
    throw std::runtime_error("unable to configure serial port");
  _x=0;
  _y=0;
  _theta=0;
  _des_rv = 0;
  _des_tv = 0;
  _state = Unsync;
}

void CapybaraRobot::disconnect(){
  if (_serial_fd>-1){
    close(_serial_fd);
    _serial_fd = -1;
  }			       
}


// TODO: Check profundly
void CapybaraRobot::processPacket(){
  _packet_buffer[_packet_buffer_idx]=0;
  int rticks, lticks;
  printf("packet: [%s]", _packet_buffer);
  sscanf(_packet_buffer, "%d %d", &rticks, &lticks);
  printf("rticks: %d, lticks: %d\n", rticks, lticks);


  // do the odometry processing:
  // 1. convert dl/dr into meters on the ground
  double dr = _right_meters_per_tick *rticks;
  double dl = _left_meters_per_tick *lticks;


  // 2. compute the delta_theta;
  double dth=(dr-dl)/_baseline;

  // 3. compute the lenght of the arc traveled
  double dist = .5*(dr+dl);
    
  // 4. compute the transform of the robot, since the last measurement
  double dx=0, dy=0;
  if (fabs(dth)<1e-4){
    dx = dist;
  } else {
    double r = dist/dth;
    dx = r*sin(dth);
    dy = r*(1-cos(dth));
  }

  // 5. update the odometry, by applying the transform (dx, dy, delta_theta) to the old transform;

  double s = sin(_theta);
  double c = cos(_theta);
  _x+=c*dx-s*dy;
  _y+=s*dx+c*dy;
  _theta += dth;
  _theta = fmod(_theta+4*M_PI, 2*M_PI);
  if (_theta>M_PI)
    _theta -= 2*M_PI;
  

  
  // assemble the velocity command of the last buffered velocity
  
  // 1. compute the linear distance traveled by the center of mass
  dist = _des_tv*_interval;
  dth =  _des_rv*_interval;
  dr = 0;
  dl = 0;
  // 2. compute the motion of each wheel
  if (fabs(dist)<1e-4) {
    dr = .5*_baseline*dth;
    dl = -.5*_baseline*dth;
  } else if (fabs(dth)<1e-4){
    dr = dist;
    dl = dist;
  } else {
    double r=dist/dth;
    dr = (r+.5*_baseline)*dth;
    dl = (r-.5*_baseline)*dth;
  }
  // 3. convert it to ticks and direction
  int dir_left = dl>0;
  int ticks_left = abs(dl)/_left_meters_per_tick;
  int dir_right = dr>0;
  int ticks_right = abs(dr)/_right_meters_per_tick;

  // 4. assemble and send the packet
  char packet_to_send[1024];
  char footer = '%';
  int n = sprintf(packet_to_send, "$01 %d %d %d %c \% \n\n", dir_left, dir_right, ticks_left, ticks_right, footer);
  cerr << "sending: [" << packet_to_send << "]" << endl;
  int sent = 0;
  while (sent<n){
    sent = write(_serial_fd,packet_to_send+sent, n-sent);
  }
}
 
void CapybaraRobot::spinOnce() {
  int n= read (_serial_fd, _serial_buffer, 255);
  for (int i = 0; i<n; i++){
    char c = _serial_buffer[i];
    switch (_state) {
    case Unsync:
      if (c=='#'){
	_state = Payload;
	_packet_buffer_idx = 0;
      }
      break;
    case Payload:
      if (c=='@'){
	_state = Unsync;
	processPacket();
	_packet_buffer_idx = 0;
      } else if (c=='#'){
	_state = Payload;
	_packet_buffer_idx = 0;
      } else {
	_packet_buffer[_packet_buffer_idx] = c;
	_packet_buffer_idx ++;
      }
      break;
    }
  }
}

