#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <stdio.h>
#include <iostream>
#include "OniSampleUtilities.h"
#include <OpenNI.h>
#include <stdlib.h> 
#include <string>

using namespace openni;
using namespace std;



int main(int argc, char **argv){

	string topic;
	string frame_id;
	int resolution;
	int rate;
	int _depth;
	int _rgb;
	int _registration;
	int _sync;

	printf("starting\n");
	fflush(stdout);
	ros::init(argc, argv, "xtionberry",ros::init_options::AnonymousName);
	ros::NodeHandle n("~");

	//Base topic name
	n.param("topic", topic, string("/xtionberry"));
	//Resolution
	//0 = 160x120
	//1 = 320x240
	n.param("resolution", resolution, 1);
	n.param("depth", _depth, 1);
	n.param("sync", _sync, 0);
	n.param("rgb", _rgb, 0);
	n.param("registration", _registration,0);
	n.param("rate", rate, 30);
	n.param("frame_id", frame_id, string("/xtionberry"));

	if(_rgb==0 && _depth==0){
		printf("You should consider to request at least one sensor duuuuuude\n");
		fflush(stdout);
	}

	printf("Launched with params:\n");
	printf("_topic:= %s\n",topic.c_str());
	printf("_depth:= %d\n",_depth);
	printf("_sync:= %d\n",_sync);
	printf("_registration:= %d\n",_registration);
	printf("_rgb:= %d\n",_rgb);
	printf("_resolution:= %d\n",resolution);
	printf("_rate:= %d\n",rate);
	printf("_frame_id:= %s\n",frame_id.c_str());
	fflush(stdout);
	//IMAGE
	ros::Publisher xtionberry_pub_depth = n.advertise<sensor_msgs::Image>("/"+topic+"/depth/image_raw", 1);
	ros::Publisher xtionberry_pub_color = n.advertise<sensor_msgs::Image>("/"+topic+"/rgb/image_raw", 1);
	//CAMERA INFO	
	ros::Publisher xtionberry_pub_CameranInfoDepth = n.advertise<sensor_msgs::CameraInfo>("/"+topic+"/depth/camera_info", 1);
	ros::Publisher xtionberry_pub_CameranInfoColor = n.advertise<sensor_msgs::CameraInfo>("/"+topic+"/rgb/camera_info", 1);
	ros::Rate loop_rate(rate);

	

	
	//OPENNI2 STUFF
	//===================================================================
	VideoStream depth;
	VideoStream color;
	openni::VideoStream** streams;
	streams = new openni::VideoStream*[2];
	streams[0]=&depth;
	streams[1]=&color;

	Status rc = OpenNI::initialize();
	if (rc != STATUS_OK)
	{
		printf("Initialize failed\n%s\n", OpenNI::getExtendedError());
		fflush(stdout);
		return 1;
	}

	Device device;
	rc = device.open(ANY_DEVICE);
	if (rc != STATUS_OK){
		printf("Couldn't open device\n%s\n", OpenNI::getExtendedError());
		fflush(stdout);
		return 2;
	}

	if(_depth==1){
		if (device.getSensorInfo(SENSOR_DEPTH) != NULL){
			rc = depth.create(device, SENSOR_DEPTH);
			if (rc != STATUS_OK){
				printf("Couldn't create depth stream\n%s\n", OpenNI::getExtendedError());
				fflush(stdout);
				return 3;
			}
		}
		

	}

	if(_rgb==1){
		if (device.getSensorInfo(SENSOR_COLOR) != NULL){
			rc = color.create(device, SENSOR_COLOR);
			if (rc != STATUS_OK){
				printf("Couldn't create rgb stream\n%s\n", OpenNI::getExtendedError());
				fflush(stdout);
				return 3;
			}
		}

	}

	if(_depth==1 && _rgb==1 && _registration==1){
		device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
	}

	if(_depth==1 && _rgb==1 && _sync==1){
		device.setDepthColorSyncEnabled(true);
	}
	
	if(resolution==1){
		if(_depth){
			rc = depth.setVideoMode(device.getSensorInfo(SENSOR_DEPTH)->getSupportedVideoModes()[0]);
		}
	}
	if(_depth){
		depth.setMirroringEnabled(false);
		rc = depth.start();
	}
	if(_rgb){
		color.setMirroringEnabled(false);
		rc = color.start();
	}

	VideoFrameRef frame;
	VideoFrameRef colorframe;
	int changedStreamDummy;
	VideoStream* pStream;
	VideoStream* pColorStream;

	sensor_msgs::CameraInfo colorInfo;
	sensor_msgs::CameraInfo depthInfo;

	sensor_msgs::Image image;
	image.header.frame_id=frame_id;
	image.is_bigendian=1;

	while (ros::ok()){
			openni::OpenNI::waitForAnyStream(streams, 2, &changedStreamDummy);
			switch (changedStreamDummy)
			{
			//DEPTH
			case 0:
				depth.readFrame(&frame);
				depthInfo.header.stamp=ros::Time::now();
				image.header.stamp=ros::Time::now();
				if(!frame.isValid()) break;

				depthInfo.header.frame_id="/"+frame_id+"_depth";
				depthInfo.width=frame.getWidth();
				depthInfo.height=frame.getHeight();

				depthInfo.K[0]=depthInfo.width/(2*tan(depth.getHorizontalFieldOfView()/2)); //fx
				depthInfo.K[5]=depthInfo.height/(2*tan(depth.getVerticalFieldOfView()/2));; //fy
				depthInfo.K[2]=depthInfo.width/2; //cx
				depthInfo.K[6]=depthInfo.height/2; //cy
				depthInfo.K[8]=1;

				image.header.frame_id="/"+frame_id+"_depth";
				image.height=frame.getHeight();
				image.width=frame.getWidth();
				image.encoding="mono16";
				image.step=frame.getWidth()*2;
				image.data.resize(image.step*image.height);
				memcpy((char*)(&image.data[0]),frame.getData(),image.height*image.width*2);
				xtionberry_pub_depth.publish(image);
				xtionberry_pub_CameranInfoDepth.publish(depthInfo);
				break;
			//COLOR
			case 1:
				color.readFrame(&colorframe);
				image.header.stamp=ros::Time::now();
				colorInfo.header.stamp=ros::Time::now();
				if(!colorframe.isValid()) break;

				colorInfo.header.frame_id="/"+frame_id+"_color";
				colorInfo.width=colorframe.getWidth();
				colorInfo.height=colorframe.getHeight();

				colorInfo.K[0]=colorInfo.width/(2*tan(color.getHorizontalFieldOfView()/2)); //fx
				colorInfo.K[5]=colorInfo.height/(2*tan(color.getVerticalFieldOfView()/2));; //fy
				colorInfo.K[2]=colorInfo.width/2; //cx
				colorInfo.K[6]=colorInfo.height/2; //cy
				colorInfo.K[8]=1;
	
				image.header.frame_id="/"+frame_id+"_color";
				image.height=colorframe.getHeight();
				image.width=colorframe.getWidth();
				image.encoding="rgb8";
				image.step=colorframe.getWidth()*3;
				image.data.resize(image.step*image.height);
				memcpy((char*)(&image.data[0]),colorframe.getData(),image.height*image.width*3);
				xtionberry_pub_color.publish(image);
				xtionberry_pub_CameranInfoColor.publish(colorInfo);
				break;
			default:
				printf("Error in wait\n");
			}
			ros::spinOnce();
			loop_rate.sleep();
	}

	depth.stop();
	depth.destroy();
	color.stop();
	color.destroy();
	device.close();
	OpenNI::shutdown();	
	return 0;
}

