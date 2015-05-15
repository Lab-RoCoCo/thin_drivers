#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include "thin_pioneer_library/pioneer_robot.h"
using namespace std;

PioneerRobot robot;
volatile double tv = 0, rv = 0;
void commandVelCallback(const geometry_msgs::TwistConstPtr twist){
  tv = twist->linear.x;
  rv = twist->angular.z;
}

int main(int argc, char** argv) {
  std::string serial_device;
  std::string odom_topic;
  std::string odom_frame_id;
  std::string command_vel_topic;
  std::string robot_type;
  
  ros::init(argc, argv, "pioneer_node");
  ros::NodeHandle nh("~");
  nh.param("serial_device", serial_device, std::string("/dev/ttyUSB0"));
  nh.param("odom_topic", odom_topic, std::string("/odom"));
  nh.param("command_vel_topic", command_vel_topic, std::string("/cmd_vel"));
  nh.param("odom_frame_id", odom_frame_id, std::string("/odom"));
  nh.param("robot_type", robot_type, std::string("p3at"));

  cerr << "running with params: ";
  cerr << "serial_device: " << serial_device << endl;
  cerr << "odom_topic: " << odom_topic << endl;
  cerr << "odom_frame_id: " << odom_frame_id << endl;
  cerr << "command_vel_topic: " << command_vel_topic << endl;
  cerr << "robot type: " << robot_type << endl;

  ros::Subscriber command_vel_subscriber = nh.subscribe<geometry_msgs::TwistConstPtr>(command_vel_topic, 1, &commandVelCallback);
  ros::Publisher odom_publisher = nh.advertise<nav_msgs::Odometry>(odom_topic, 1);
  robot.connect(robot_type, serial_device);
  nav_msgs::Odometry odom;
  odom.header.frame_id = odom_frame_id;
  int seq = 0;
  int _packet_count = 0;
  while(ros::ok()){
    ros::spinOnce();
    robot.spinOnce();
    robot.setSpeed(tv,rv);
    // send the odometry
    double x,y,theta;
    robot.getOdometry(x,y,theta);
    odom.header.seq = seq;
    odom.header.stamp = ros::Time::now();
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0;
    double s = sin (theta/2);
    double c = cos (theta/2);
    odom.pose.pose.orientation.x = 0;
    odom.pose.pose.orientation.y = 0;
    odom.pose.pose.orientation.z = s;
    odom.pose.pose.orientation.w = c;
    odom_publisher.publish(odom);
    seq++;
  }
  robot.disconnect();
}
