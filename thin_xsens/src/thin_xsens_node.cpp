#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <sstream>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include <xsens/xsportinfoarray.h>
#include <xsens/xsdatapacket.h>
#include <xsens/xstime.h>
#include <legacydatapacket.h>
#include <int_xsdatapacket.h>
#include <enumerateusbdevices.h>

#include "deviceclass.h"

// TODO Lista dei 
int main ( int argc, char** argv )
{
  std::string device_id;
  bool calibrated_data_on, orientation_data_on, magnetic_data_on;
  std::string frame_id;
  std::string topic_name;

  ros::init ( argc, argv, "thin_xsens_node" );
  ros::NodeHandle nh ( "~" );
  nh.param ( "device_id", device_id, std::string ( "0" ) );
  nh.param ( "calibrated_data_on", calibrated_data_on, true );
  nh.param ( "orientation_data_on", orientation_data_on, true );
  nh.param ( "magnetic_data_on", magnetic_data_on, true );
  nh.param ( "frame_id", frame_id, std::string ( "/imu" ) );
  nh.param ( "topic_name", topic_name, std::string ( "/imu" ) );

  std::cerr << "Running with params: "<< std::endl<< std::endl;
  
  std::cout << "device_id: " << device_id << std::endl;
  std::cout << "calibrated_data_on: " << (calibrated_data_on?"true":"false") << std::endl;
  std::cout << "orientation_data_on: " << (orientation_data_on?"true":"false") << std::endl;
  std::cout << "magnetic_data_on: " << (magnetic_data_on?"true":"false") << std::endl;
  std::cout << "frame_id: " << frame_id << std::endl;
  std::cout << "topic_name: " << topic_name<< std::endl<< std::endl;
  
  ros::Publisher imu_publisher, mag_publisher;

  std::stringstream imu_topic, magnetic_topic;
  imu_topic<<topic_name;
  imu_topic<<std::string("/data");
  magnetic_topic<<topic_name;
  magnetic_topic<<std::string("/magnetic");
  
  imu_publisher = nh.advertise<sensor_msgs::Imu> ( imu_topic.str(), 1 );
  if( magnetic_data_on )
    mag_publisher = nh.advertise<sensor_msgs::MagneticField> ( magnetic_topic.str(), 1 );

  if( !calibrated_data_on && !orientation_data_on )
    orientation_data_on = "1";
  
  DeviceClass device;

  try
  {
    // Scan for connected USB devices
    std::cout << "Scanning for USB devices..." << std::endl;
    XsPortInfoArray portInfoArray;
    xsEnumerateUsbDevices ( portInfoArray );
    if ( !portInfoArray.size() )
      throw std::runtime_error ( "No device found. Aborting." );

    XsPortInfo mtPort;
    bool device_found = false;
    if ( device_id.compare("0") )
    {
      for ( int i = 0; i < portInfoArray.size(); i++ )
      {
        mtPort = portInfoArray.at ( i );
        if ( !mtPort.deviceId().toString().toStdString().compare(device_id) )
        {
          device_found = true;
          break;
        }
      }
    }
    else
    {
      // Use the first detected device
      mtPort = portInfoArray.at ( 0 );
      device_found = true;
    }

    if ( !device_found )
      throw std::runtime_error ( "No device with the required ID found. Aborting." );

    // Open the port with the detected device
    std::cout << "Opening port..." << std::endl;
    if ( !device.openPort ( mtPort ) )
      throw std::runtime_error ( "Could not open port. Aborting." );

    // Put the device in configuration mode
    std::cout << "Putting device into configuration mode..." << std::endl;
    if ( !device.gotoConfig() ) // Put the device into configuration mode before configuring the device
    {
      throw std::runtime_error ( "Could not put device into configuration mode. Aborting." );
    }

    // Request the device Id to check the device type
    mtPort.setDeviceId ( device.getDeviceId() );

    // Check if we have an MTmk4 device
    if ( !mtPort.deviceId().isMtMk4() )
    {
      throw std::runtime_error ( "No device found. Aborting." );
    }
    std::cout << "Found a device with id: " << mtPort.deviceId().toString().toStdString() 
              << " @ port: " << mtPort.portName().toStdString() << std::endl;

    try
    {
      // Print information about detected MTmk4 device
      std::cout << "Device: " << device.getProductCode().toStdString() << " opened." << std::endl;

      // Configure the device. Note the differences between MTix and MTmk4
      std::cout << "Configuring the device..." << std::endl;

      XsOutputConfigurationArray configArray;
      
      if( orientation_data_on )
      {
        XsOutputConfiguration quat ( XDI_Quaternion, 100 );
        configArray.push_back ( quat );
      }
      
      if( calibrated_data_on )
      {
        XsOutputConfiguration acc ( XDI_Acceleration, 100 );
        configArray.push_back ( acc );
        XsOutputConfiguration gyro ( XDI_RateOfTurn, 100 );
        configArray.push_back ( gyro );
      }
      
      if( magnetic_data_on )
      {
        XsOutputConfiguration mag ( XDI_MagneticField, 100 );
        configArray.push_back ( mag );
      }
      
      if ( !device.setOutputConfiguration ( configArray ) )
        throw std::runtime_error ( "Could not configure MTmk4 device. Aborting." );
      
      
      // Put the device in measurement mode
      std::cout << "Putting device into measurement mode..." ;
      if ( !device.gotoMeasurement() )
        throw std::runtime_error ( "Could not put device into measurement mode. Aborting." );

      std::cout << "done! Start streaming" << std::endl;
      XsByteArray data;
      XsMessageArray msgs;
      
      while ( ros::ok() )
      {      
        device.readDataToBuffer ( data );
        
        sensor_msgs::Imu imu_data;
        imu_data.header.frame_id = frame_id;
        imu_data.header.stamp = ros::Time::now();
        
        sensor_msgs::MagneticField mag_data;
        mag_data.header.frame_id = frame_id;
        mag_data.header.stamp = imu_data.header.stamp;
        
        device.processBufferedData ( data, msgs );
        for ( XsMessageArray::iterator it = msgs.begin(); it != msgs.end(); ++it )
        {
          // Retrieve a packet
          XsDataPacket packet;
          packet.setMessage ( ( *it ) );
          packet.setDeviceId ( mtPort.deviceId() );

          if( orientation_data_on && packet.containsOrientation() )
          {
            XsQuaternion quaternion = packet.orientationQuaternion();
            imu_data.orientation.x = quaternion.m_x;
            imu_data.orientation.y = quaternion.m_y;
            imu_data.orientation.z = quaternion.m_z;
            imu_data.orientation.w = quaternion.m_w;
          }
          
          if( calibrated_data_on && 
              packet.containsCalibratedAcceleration() && packet.containsCalibratedGyroscopeData() )
          {
            XsVector acceleration = packet.calibratedAcceleration();
            XsVector gyro = packet.calibratedGyroscopeData();
            
            imu_data.linear_acceleration.x = acceleration.at ( 0 );
            imu_data.linear_acceleration.y = acceleration.at ( 1 );
            imu_data.linear_acceleration.z = acceleration.at ( 2 );
            
            imu_data.angular_velocity.x = gyro.at ( 0 );
            imu_data.angular_velocity.y = gyro.at ( 1 );
            imu_data.angular_velocity.z = gyro.at ( 2 );
            
          }
          
          imu_publisher.publish ( imu_data );
          
          if( magnetic_data_on && packet.containsCalibratedMagneticField() )
          {
            XsVector magnetic = packet.calibratedMagneticField();
           
            mag_data.magnetic_field.x = magnetic.at( 0 );
            mag_data.magnetic_field.y = magnetic.at( 1 );
            mag_data.magnetic_field.z = magnetic.at( 2 );
            
            mag_publisher.publish ( mag_data );
          }
        }
        msgs.clear();
        ros::spinOnce();
        // TODO Check here!!!
        //XsTime::msleep ( 1.0 );
        XsTime::msleep ( 0 );
      }
    }
    catch ( std::runtime_error const & error )
    {
      std::cout << error.what() << std::endl;
    }
    catch ( ... )
    {
      std::cout << "An unknown fatal error has occured. Aborting." << std::endl;
    }
    // Close port
    std::cout << "Closing port..." << std::endl;
    device.close();
  }
  catch ( std::runtime_error const & error )
  {
    std::cout << error.what() << std::endl;
  }
  catch ( ... )
  {
    std::cout << "An unknown fatal error has occured. Aborting." << std::endl;
  }

  std::cout << "Successful exit." << std::endl;

  return 0;
}
