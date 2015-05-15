#include <ros/ros.h>
#include <string.h>
#include <sensor_msgs/LaserScan.h>
#include "hokuyo.h"
#include <iostream>

using namespace std;

static HokuyoLaser urg;
static sensor_msgs::LaserScan scan;
char buf[HOKUYO_BUFSIZE];

int main(int argc, char** argv) {
  std::string topic, frame_id, serial_port, model;
  ros::init(argc, argv, "xtion",ros::init_options::AnonymousName);
  ros::NodeHandle n("~");
  n.param("topic", topic, std::string("/scan"));
  n.param("frame_id", frame_id, std::string("/laser_frame"));
  n.param("serial_port", serial_port, std::string("/dev/ttyACM0"));
  n.param("model", model, std::string("utm"));

  cerr << "running with params: " << endl;
  cerr << "_serial_port: " << serial_port << endl;
  cerr << "_frame_id: " << frame_id << endl;
  cerr << "_topic: " << topic << endl;
  cerr << "_model: " << model << endl;


  int max_int_range = 0;
  int max_beams = 0;
    
  if (model == "urg") {
    max_int_range = URG_MAX_RANGE;
    max_beams = URG_MAX_BEAMS;
    scan.angle_increment = URG_ANGULAR_STEP;
  } else if (model == "ubg") {
    max_int_range = UBG_MAX_RANGE;
    max_beams = UBG_MAX_BEAMS;
    scan.angle_increment = UBG_ANGULAR_STEP;
  } else if (model == "utm") {
    max_int_range = UTM_MAX_RANGE;
    max_beams = UTM_MAX_BEAMS;
    scan.angle_increment = UTM_ANGULAR_STEP;
  } else {
    cerr << "unknwn model, aborting" << endl;
    return 0;
  }
  scan.ranges.resize(max_beams);
  scan.angle_min = -scan.angle_increment * max_beams/2;
  scan.angle_max = scan.angle_increment * max_beams/2;
  scan.range_min = 1e-3*max_int_range;
  // HACK
  scan.range_max = 0.02;
  scan.time_increment = 0;
  scan.scan_time = 0;

  ros::Publisher pub = n.advertise<sensor_msgs::LaserScan>(topic, 10);

  int o=hokuyo_open_usb(&urg, serial_port.c_str());
  if (o<=0) {
    cerr << "failure in opening serial port" << endl;
    return -1;
  }

  o=hokuyo_init(&urg,1);
  if (o<=0){
    cerr << "failure in initializing device" << endl;
    return -1;
  }

  o=hokuyo_startContinuous(&urg, 0, urg.maxBeams, 0, 0);
  if (o<=0){
    cerr << "failure in starting continuous mode" << endl;
    return -1;
  }

  cerr << "device started" << endl;

  scan.header.seq = 0;
  scan.header.frame_id = frame_id;

 
  while (ros::ok()){
    hokuyo_readPacket(&urg, buf, HOKUYO_BUFSIZE,10);
    scan.header.stamp = ros::Time::now();
    HokuyoRangeReading reading;
    hokuyo_parseReading(&reading, buf, 0);

    for (int i=0; i<reading.n_ranges; i++){
      if (reading.ranges[i]>20 && reading.ranges[i]< 5601)
	scan.ranges[i]=1e-3*reading.ranges[i];
      else
	scan.ranges[i]=scan.range_max;
    }
    pub.publish(scan);
  } // while run
  
  printf("Closing.\n");
  hokuyo_close(&urg);
  return 0;

  
}
