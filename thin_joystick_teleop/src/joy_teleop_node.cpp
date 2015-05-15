#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <ros/ros.h>
#include <linux/joystick.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
using namespace std;

std::string joy_device;
std::string command_vel_topic;
double max_tv;
double max_rv;
int tv_axis;
int rv_axis;
int boost_button;
int halt_button;

int main(int argc, char** argv) {
  ros::init(argc, argv, "joy_control");
  ros::NodeHandle nh("~");
  nh.param("joy_device", joy_device, std::string("/dev/input/js0"));
  nh.param("command_vel_topic", command_vel_topic, std::string("/cmd_vel"));
  nh.param("max_tv", max_tv, 0.5);
  nh.param("max_rv", max_rv, 1.0);
  nh.param("tv_axis", tv_axis, 1);
  nh.param("rv_axis", rv_axis, 3);
  nh.param("boost_button", boost_button, 4);
  nh.param("halt_button", halt_button, 5);

  cerr << "running with arguments: " << endl << endl;
  cerr << "joy_device: " << joy_device << endl;
  cerr << "command_vel_topic: " << command_vel_topic << endl;
  cerr << "max_tv: " << max_tv << endl;
  cerr << "max_rv: " << max_rv << endl;
  cerr << "rv_axis: " << rv_axis << endl;
  cerr << "tv_axis: " << tv_axis << endl;
  cerr << "boost_button: " << boost_button << endl;
  cerr << "halt_button: " << halt_button << endl;
 
  int fd = open (joy_device.c_str(), O_RDONLY|O_NONBLOCK);
  if (fd<0) {
    cerr << "no joy found" << endl;
  }
  ros::Publisher cmd_vel_publisher = nh.advertise<geometry_msgs::Twist>(command_vel_topic, 1);
  ros::Rate r(50);

  float tv = 0;
  float rv = 0;
  float tvscale = max_tv/32767.0;
  float rvscale = max_rv/32767.0;
  float gain = 1;
  
  geometry_msgs::Twist twist;
  while(ros::ok()){
    struct js_event e;
    while (read (fd, &e, sizeof(e)) > 0 ){
      int axis = e.number;
      int value = e.value;
      int type = e.type;
      if (axis == tv_axis && type == 2) {
	tv = -value * tvscale;
      }
      if (axis == rv_axis && type == 2) {
	rv = -value *rvscale;
      }
      if (axis == halt_button && type==1){
	tv = 0;
	rv = 0;
      } else if (axis == boost_button && type == 1) {
	if(value)
	  gain = 2.0;
	else
	  gain = 1.0;
      } 
    }
    twist.linear.x = tv*gain;
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = rv*gain;
    cmd_vel_publisher.publish(twist);
    ros::spinOnce();
    r.sleep();
  }
}
