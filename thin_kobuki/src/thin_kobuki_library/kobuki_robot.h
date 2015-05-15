#pragma once
#include <stdint.h>
#include <string>
#include <istream>

class Packet;
class PacketSyncFinder;
class PacketParser;

class KobukiRobot {
public:
  enum Side {Left, Center, Right};
  KobukiRobot();
  void connect(std::string device);
  void disconnect();
  void runFromFile(std::istream& is);  
  void spinOnce();
  void playSequence(uint8_t sequence);
  void playSound(uint8_t duration, uint16_t note);
  void setSpeed(double tv, double rv); // tv: meters/s, rv:radians/s
  int packetCount() const {return _packet_count;}
  

  /**  Accessor ,methods */
  void getOdometry(double& x, double& y, double& theta) const;
  bool bumper(Side s) const;
  bool cliff(Side s) const;
  bool wheelDrop(Side s) const;
  int pwm(Side s) const;   //0-255
  bool button(int num) const;
  bool charger() const;
  float battery() const;   //percentage
  bool overcurrent(Side s) const ;
  float current(Side s) const ;   //ampere
  int cliffData(Side s) const ;

  /**
    Flag will be setted when signal is detected
    0x01 for NEAR_LEFT state
    0x02 for NEAR_CENTER state
    0x04 for NEAR_RIGHT state
    0x08 for FAR_CENTER state
    ox10 for FAR_LEFT state
    0x20 for FAR_RIGHT state 
  */
  int dockingData(Side s) const;
  float analogGPIO(int channel) const ;   // volts, max 3.3 
  uint16_t digitalGPIO() const;    // first 4 bits */ 

  /*



  /*
  
  
  inline float gyroAngle();
  inline float gyroRate();
  */

protected:   
  uint16_t _timestamp;
  uint8_t _bumper;
  uint8_t _wheel_drop;
  uint8_t _cliff;
  uint16_t _left_encoder, _right_encoder;
  uint8_t _left_pwm, _right_pwm;
  uint8_t _button;
  uint8_t _charger;
  uint8_t _battery;
  uint8_t _overcurrent_flags;
  uint8_t _right_docking_signal, _center_docking_signal, _left_docking_signal;
  uint16_t _right_cliff_signal, _center_cliff_signal, _left_cliff_signal;
  uint16_t inertial_rate, inertial_angle;
  uint8_t _right_motor_current, _left_motor_current;
  uint8_t _hw_patch, _hw_major, _hw_minor;
  uint8_t _fw_patch, _fw_major, _fw_minor;
  uint16_t _analog_input[4];
  uint16_t _digital_input;
  uint32_t _udid[3];
  uint32_t _P,_I,_D;
  double _x, _y, _theta;
  double _baseline, _left_ticks_per_m, _right_ticks_per_m;
  bool _first_round;
  int _packet_count;
  Packet* _currentPacket;

  void processOdometry(uint16_t left_encoder_, uint16_t right_encoder_);
  void processPacket(Packet* p);

  int _serial_fd;
  PacketSyncFinder* _sync_finder;
  PacketParser* _parser;
  Packet* _control_packet;
};
