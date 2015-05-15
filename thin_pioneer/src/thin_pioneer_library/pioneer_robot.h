#pragma once
#include <string>

class PioneerRobot {
 public:
  PioneerRobot();
  ~PioneerRobot();
  void connect(const std::string& model, const std::string& device);
  void disconnect();
  void setSpeed(double tv, double rv);
  void getOdometry(double&x, double& y, double& theta);
  void getSpeed(double&tv, double& rv);
  void spinOnce();
 protected:
  double _x, _y, _theta, _tv, _rv;
  double _des_tv, _des_rv;
  double _update_timestamp;
};

