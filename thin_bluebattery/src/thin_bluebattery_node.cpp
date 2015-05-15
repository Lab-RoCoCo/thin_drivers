#include <string>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <fstream>
#include "serial.h"
using namespace std;

class BatteryLogger {
public:
    BatteryLogger(ros::NodeHandle& nh) : nh(nh) {
        ros::NodeHandle nhp("~");
        nhp.param<string>("log", this->params.logFilename, "");
        if (this->params.logFilename != "") {
            char buf[1024];
            ROS_INFO("Logging blue battery status on %s (directory: %s).", this->params.logFilename.c_str(), getcwd(buf, 1024));
            batteryLog.open(this->params.logFilename.c_str());
        }
        string device;
        nhp.param<string>("serial_device", device, "/det/ttyACM0");
        ROS_INFO("Opening port %s...", device.c_str());
        serial_fd = serial_open(device.c_str());
        serial_set_interface_attribs(serial_fd, B9600, 0);
        if (serial_fd <= 0) {
            ROS_ERROR("Error opening port");
            exit(-1);
        }
        out.curiPub = nh.advertise<std_msgs::Float32>("blue_battery/current", 0.0);
        out.curvPub = nh.advertise<std_msgs::Float32>("blue_battery/voltage", 0.0);
        out.batPub = nh.advertise<std_msgs::Float32>("blue_battery/percentage", 1.0);
        out.usedPub = nh.advertise<std_msgs::Float32>("blue_battery/used", 0.0);
        out.totPub = nh.advertise<std_msgs::Float32>("blue_battery/total", 0.0);
        ROS_INFO("Reading...");
    }
    
    ~BatteryLogger() { }

    string com(string s) {
        serial_write(serial_fd, (s+"\n").c_str(), s.size()+1);
        string resp;
        size_t ss = 0;
        char buffer[128];
        while (true) {
            int r = serial_readline(serial_fd, buffer, 128, 1.0);
            resp = buffer;
            if (resp.substr(0, 4) != "RECV") {
                ROS_INFO("Received from Olimexino: %s", resp.c_str());
                if ((ss = resp.find("RECV")) != string::npos) {
                    resp = resp.substr(ss);
                    ROS_DEBUG("Found spurious bytes, good string starting at %zu [%s]", ss, resp.c_str());
                    break;
                }
            }
            else break;
        }
        size_t a = resp.find_first_of(':') + 1;
        resp = resp.substr(a);
        a = resp.find_first_not_of(' ');
        resp = resp.substr(a);
        resp = resp.substr(0, resp.size() - 1).c_str();
        return resp;
    }

    void spinOnce() {
        string r = com("cur*");
        if (r == "") {
            ROS_ERROR("Communication error, will retry soon");
            return;
        }
        istringstream iss(r);
        float cur_v, cur_i, bat_perc, used_coulombs, total_coulombs;
        iss >> cur_v >> cur_i >> bat_perc >> used_coulombs >> total_coulombs;
        std_msgs::Float32 obj;
        obj.data = cur_i;           out.curiPub.publish(obj);
        obj.data = cur_v;           out.curvPub.publish(obj);
        obj.data = bat_perc;        out.batPub.publish(obj);
        obj.data = used_coulombs;   out.usedPub.publish(obj);
        obj.data = total_coulombs;  out.totPub.publish(obj);
        if (batteryLog.is_open()) {
            batteryLog << fixed << ros::Time::now().toSec() << " " << r << endl;
        }
    }

protected:
    struct Out {
        ros::Publisher curiPub;
        ros::Publisher curvPub;
        ros::Publisher batPub;
        ros::Publisher usedPub;
        ros::Publisher totPub;
    } out;

    struct Params {
        std::string logFilename;
    } params;

    int serial_fd;
    ros::NodeHandle& nh;

    ofstream batteryLog;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "bluebattery");
    ros::NodeHandle nh;

    BatteryLogger batlog(nh);

    ros::Rate rate(5.0);
    while (ros::ok()) {
        ros::spinOnce();
        batlog.spinOnce();
        rate.sleep();
    }

    return 0;
}

