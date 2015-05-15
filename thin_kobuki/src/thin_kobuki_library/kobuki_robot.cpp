#include "kobuki_robot.h"
#include "kobuki_protocol.h"
#include <cmath>
#include <stdexcept>
#include <iostream>
#include "serial.h"
using namespace std;

KobukiRobot::KobukiRobot() {
  _x = 0;
  _y = 0;
  _theta = 0;
  _left_ticks_per_m = 1./11724.41658029856624751591;
  _right_ticks_per_m = 1./11724.41658029856624751591;
  _baseline = 0.230;
  _first_round = true;
  _serial_fd=-1;
  _parser = new PacketParser;
  _sync_finder  = new PacketSyncFinder;
  _control_packet = new Packet;
} 


void KobukiRobot::playSequence(uint8_t sequence) {
  SoundSequencePayload* sp = new SoundSequencePayload;
  sp->sequence = sequence;
  _control_packet->_payloads.push_back(sp);
}

void KobukiRobot::playSound(uint8_t duration, uint16_t note) {
  SoundPayload* sp = new SoundPayload;
  sp->note  = note;
  sp->duration = duration;
  _control_packet->_payloads.push_back(sp);
}

void KobukiRobot::setSpeed(double tv, double rv) {
  BaseControlPayload* bp = new BaseControlPayload;
  // convert to mm;
  tv *=1000;
  double b2 = _baseline * 500;

  if (fabs(tv) < 1){
    cerr << "pure rotation" << endl;
    bp->radius = 1;
    bp->speed =  (int16_t) (rv * b2);
  } else if (fabs(rv) < 1e-3 ) {
    cerr << "pure translation" << endl;
    bp->speed = (int16_t) tv;
    bp->radius = 0;
  } else {
    cerr << "translation and rotation" << endl;
    float r = tv/rv;
    bp->radius = (int16_t) r;
    if (r>1) {
      bp->speed = (int16_t) (tv * (r + b2)/ r);
    } else if (r<-1) {
      bp->speed = (int16_t) (tv * (r - b2)/ r);
    }
  }
  _control_packet->_payloads.push_back(bp);
}

void KobukiRobot::spinOnce() {
  if (_serial_fd < 0)
    throw std::runtime_error("robot not connected");
  
  char buf [255];
  int n = read (_serial_fd, buf, 255);
  for (int i = 0; i<n; i++){
    _sync_finder->putChar(buf[i]);
    if (_sync_finder->packetReady()){
      const unsigned char* b=_sync_finder->buffer();
      Packet* packet = _parser->parseBuffer(b, _sync_finder->bufferLength());
      if (packet) {
	_packet_count++;
	processPacket(packet);
	delete packet;
	unsigned char tx_buffer[1024];
	if (_control_packet->_payloads.size()) {
	  int k = _control_packet->write(tx_buffer);
	  if ( k>0 ) {
	    int sent = 0;
	    do {
	      sent += write(_serial_fd, tx_buffer+sent, k-sent);
	    } while (sent<k);
	  }
	  _control_packet->clear();
	}
      } 
    }
  }
}



void KobukiRobot::connect(std::string device) {
  if (_serial_fd>-1)
    disconnect();
  _serial_fd = serial_open(device.c_str());
  if (_serial_fd<0)
    throw std::runtime_error("error in opening serial port");
  int att = serial_set_interface_attribs (_serial_fd, B115200, 0);
  if (att<0) {
    throw std::runtime_error("error in setting attributes to serial port");
  }
  _packet_count = 0;
}

void KobukiRobot::runFromFile(istream& is) {
  while(is) {
    unsigned char c = is.get();
    _sync_finder->putChar(c);
    if (_sync_finder->packetReady()){
      const unsigned char* b=_sync_finder->buffer();
      Packet* packet = _parser->parseBuffer(b, _sync_finder->bufferLength());
      if (packet) {
        processPacket(packet);
	_packet_count++;
        delete packet;
      }
    }
  }
}

void KobukiRobot::disconnect() {
  if (_serial_fd>-1)
    close(_serial_fd);
  _serial_fd=-1;
}

void KobukiRobot::getOdometry(double& x, double& y, double& theta) const {
  x= _x;
  y= _y;
  theta = _theta;
}

bool KobukiRobot::bumper(Side s) const {
  switch(s){
  case Right: return _bumper&0x1;
  case Center: return _bumper&0x2;
  case Left: return _bumper&0x4;
  }
}

bool KobukiRobot::cliff(Side s) const {
  switch(s){
  case Right: return _cliff&0x1;
  case Center: return _cliff&0x2;
  case Left: return _cliff&0x4;
  }
}  

bool KobukiRobot::wheelDrop(Side s) const {
  switch(s){
  case Right: return _wheel_drop&0x1;
  case Left: return _wheel_drop&0x2;
  }
  return 0;
}

int KobukiRobot::pwm(Side s) const { 
  switch(s){
  case Right: return _right_pwm;
  case Left: return _left_pwm;
  }
  return 0;
}

bool KobukiRobot::button(int num) const {
  switch(num){
  case 0: return _button&0x1;
  case 1: return _button&0x2;
  case 2: return _button&0x4;
  }
  return 0;
}

bool KobukiRobot::charger() const {
  return _charger;
}

float KobukiRobot::battery() const {
  return 10.0f*_battery/16.7;
}

bool KobukiRobot::overcurrent(Side s) const {
  switch(s){
  case Right: return _overcurrent_flags&0x1;
  case Left: return _overcurrent_flags&0x2;
  }
  return 0;
}

float KobukiRobot::current(Side s) const {
  switch(s){
  case Right: return _right_motor_current*0.01;
  case Left: return _left_motor_current*0.01;
  }
  return 0;
}

int KobukiRobot::cliffData(Side s) const {
  switch(s){
  case Right: return _right_cliff_signal;
  case Center: return _center_cliff_signal;
  case Left: return _left_cliff_signal;
  }
}

int KobukiRobot::dockingData(Side s) const {
  switch(s){
  case Right: return _right_docking_signal;
  case Center: return _center_docking_signal;
  case Left: return _left_docking_signal;
  }
}

float KobukiRobot::analogGPIO(int channel) const {
  return _analog_input[channel]*3.3/4095;
}

uint16_t KobukiRobot::digitalGPIO() const {
  return _digital_input;
}

void KobukiRobot::processOdometry(uint16_t left_encoder_, uint16_t right_encoder_){
  if (!_first_round) {
    double dl= _left_ticks_per_m * (left_encoder_-_left_encoder);
    double dr= _right_ticks_per_m * (right_encoder_-_right_encoder);
    double dx = 0, dy = 0, dtheta = (dr-dl)/_baseline;
    if (dl!=dr) {
      double R=.5*(dr+dl)/dtheta;
      dx = R*sin(dtheta);
      dy = R*(1-cos(dtheta));
    } else {
      dx = dr;
    } 
    double s = sin(_theta), c = cos(_theta);
    _x += c * dx - s * dy;
    _y += s * dx + c * dy;
    _theta += dtheta;
    _theta = fmod(_theta+4*M_PI, 2*M_PI);
    if (_theta>M_PI)
      _theta -= 2*M_PI;
  } else {
    _x = _y = _theta = 0;
  }
  _left_encoder = left_encoder_;
  _right_encoder = right_encoder_;
}

void KobukiRobot::processPacket(Packet* p) {
  for (size_t i = 0; i<p->_payloads.size(); i++){
    SubPayload *bp = p->_payloads[i];
    switch(bp->header()) {
      // BaseSensorDataPayload:
    case BasicSensorDataPayload_HEADER: {
      BasicSensorDataPayload* bsdp = static_cast<BasicSensorDataPayload*>(bp);
      _timestamp = bsdp->timestamp;
      _wheel_drop = bsdp->wheel_drop;
      _bumper = bsdp->bumper;
      _cliff = bsdp->cliff;
      _left_pwm = bsdp->left_pwm;
      _right_pwm =bsdp->right_pwm;
      _button = bsdp->button;
      _charger = bsdp->charger;
      _battery = bsdp->battery;
      _overcurrent_flags = bsdp->overcurrent_flags;
      processOdometry(bsdp->left_encoder, bsdp->right_encoder);
    }
      break;
    case DockingIRPayload_HEADER: {
      DockingIRPayload* dirdp = static_cast<DockingIRPayload*>(bp);
      _right_docking_signal = dirdp->right_signal;
      _center_docking_signal = dirdp->center_signal;
      _left_docking_signal = dirdp->left_signal;
    }
      break;
	
    case InertialSensorDataPayload_HEADER: {
      InertialSensorDataPayload* idp = static_cast<InertialSensorDataPayload*>(bp);
      inertial_rate = idp->rate;
      inertial_angle = idp->angle;
    }
      break;
    case CliffSensorDataPayload_HEADER: {
      CliffSensorDataPayload* csdp = static_cast<CliffSensorDataPayload*>(bp);
      _right_cliff_signal = csdp->right_signal;
      _center_cliff_signal = csdp->center_signal;
      _left_cliff_signal = csdp->left_signal;
    }
      break;
    case CurrentPayload_HEADER: {
      CurrentPayload* cp = static_cast<CurrentPayload*>(bp);
      _right_motor_current = cp->right_motor;
      _left_motor_current = cp->left_motor;
    }
      break;
    case HardwareVersionPayload_HEADER: {
      HardwareVersionPayload* hwp = static_cast<HardwareVersionPayload*>(bp);
      _hw_patch = hwp->patch;
      _hw_major = hwp->major;
      _hw_minor = hwp->minor;
    }
      break;
    case FirmwareVersionPayload_HEADER: {
      FirmwareVersionPayload* fwp = static_cast<FirmwareVersionPayload*>(bp);
      _fw_patch = fwp->patch;
      _fw_major = fwp->major;
      _fw_minor = fwp->minor;
    }
      break;
    case GyroPayload_HEADER: break;
    case GPIOPayload_HEADER: {
      GPIOPayload* gpiop = static_cast<GPIOPayload*>(bp);
      for (int k=0; i<4; i++)
	_analog_input[k] = gpiop->analog_input[k];
      _digital_input = gpiop->digital_input;
    }
      break;
    case UUIDPayload_HEADER: {
      UUIDPayload* uuidp = static_cast<UUIDPayload*>(bp);
      for (int k=0; i<4; i++)
	_udid[k] = uuidp->udid[k];
    }
      break;
    case ControllerInfoPayload_HEADER: {
      ControllerInfoPayload* cip = static_cast<ControllerInfoPayload*>(bp);
      _P=cip->P;
      _I=cip->I;
      _D=cip->D;
    }
      break;
    default:
      ;
    }
  }
  _first_round = false;
}

