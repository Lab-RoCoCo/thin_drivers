#pragma once
#include <string>

class CapybaraRobot{
public:
  CapybaraRobot();
  void connect(const std::string& device);
  void disconnect();			       
  void spinOnce();
  inline void getOdometry(double& x, double& y, double& theta) {
    x = _x;
    y = _y;
    theta = _theta;
  }
  inline void setSpeed(double tv, double rv);
protected:
  double _interval; // in seconds between updates
  double _baseline;
  double _left_meters_per_tick;
  double _right_meters_per_tick;
  int _serial_fd;
  double _x, _y, _theta;
  double _des_tv, _des_rv;
  int _des_left_ticks, _des_right_ticks;
  enum State {Unsync, Payload};
  State _state;
  char _serial_buffer[1024];
  char _packet_buffer[1024];
  int _packet_buffer_idx;
  void processPacket();
};
