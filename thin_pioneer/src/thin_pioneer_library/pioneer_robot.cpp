#include "pioneer_robot.h"
#include "pioneer_lib.h"
#include <cmath>
#include <iostream>

using namespace std;

PioneerRobot::PioneerRobot(){
  _x=_y=_theta=_tv=_rv=_des_tv=_des_rv=0;
  _update_timestamp=0;
}

PioneerRobot::~PioneerRobot(){
  disconnect();
  _x=_y=_theta=_tv=_rv=_des_tv=_des_rv=0;
}

void PioneerRobot::connect(const std::string& model, const std::string& device) {
  carmen_base_direct_initialize_robot(model.c_str(), device.c_str());
  carmen_base_direct_sonar_off();
  carmen_base_direct_set_acceleration(1);
  carmen_base_direct_set_deceleration(5);
}

void PioneerRobot::disconnect() {
  carmen_base_direct_shutdown_robot();
}

void PioneerRobot::setSpeed(double tv, double rv) {
  _des_tv=tv;
  _des_rv=rv;
  carmen_base_direct_set_velocity(_des_tv, _des_rv);
}

void PioneerRobot::getOdometry(double&x, double& y, double& theta){
  x=_x;
  y=_y;
  theta=_theta;
}

void PioneerRobot::getSpeed(double&tv, double& rv){
  tv=_tv;
  rv=_rv;
}

void PioneerRobot::spinOnce(){
  carmen_base_direct_update_status(&_update_timestamp);

  double displacement, dtheta;
  double dx=0, dy=0;
  carmen_base_direct_get_state(&displacement, &dtheta, &_tv, &_rv);

  if (fabs(dtheta)<1e-9){
    dx = displacement;
    dtheta=0;
  } else {
    double r = displacement/dtheta;
    dy=r*sin(dtheta);
    dx=r*(1-cos(dtheta));
  }
  
  // get the dx, dy and dtheta;
  double s = sin(_theta);
  double c = cos(_theta);
  _x+=c*dx-s*dy;
  _y+=s*dx+c*dy;
  _theta+=dtheta;

  _theta = fmod(_theta+4*M_PI, 2*M_PI);
  if (_theta>M_PI)
    _theta -= 2*M_PI;
}
