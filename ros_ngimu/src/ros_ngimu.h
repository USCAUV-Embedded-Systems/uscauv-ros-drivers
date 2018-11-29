//
// Created by jamie on 10/24/18.
//

#ifndef ROS_NGIMU_H
#define ROS_NGIMU_H

#include <asio.hpp>
#include <memory>
#include <utility>

#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <std_msgs/Header.h>

#include <ros/ros.h>

#include "Osc99.h"

namespace ros_ngimu
{

// shim to allow shared_ptr to work with arrays
// from https://stackoverflow.com/questions/13061979/shared-ptr-to-an-array-should-it-be-used
template< typename T >
struct array_deleter
{
	void operator ()( T const * p)
	{
		delete[] p;
	}
};

class ros_ngimu
{

	// ASIO variables
	asio::io_service io_service;
	asio::serial_port serialPort;

	// data for OSC library
	OscSlipDecoder oscSlipDecoder;

	// only one byte, weird stuff happened when I tried to use a larger buffer
	uint8_t serialBuffer;

	// publishers
	ros::Publisher eulerPublisher;
	ros::Publisher quaternionPublisher;
	ros::Publisher accelPublisher;

	// messages (declared here, and filled in with new data each loop)
	geometry_msgs::Vector3Stamped eulerAngles;
	geometry_msgs::QuaternionStamped quaternion;
	geometry_msgs::Vector3Stamped earthAccel;

	// ASIO event handler for characters read
	void onSerialRead(const asio::error_code& errorCode, std::size_t bytes_transferred);

	// Prepare a ROS message header for sending the the next message
	void prepareHeader(std_msgs::Header & header);



public:

	// construct with serial port
	ros_ngimu(std::string const & serialPortPath, ros::NodeHandle & node);

	// callback for OSC packets
	void onGetPacket(OscPacket * const oscPacket);

	// callback for OSC messages
	void onGetMessage(const OscTimeTag * const oscTimeTag, OscMessage * const oscMessage);

	// run event loop (never returns until node is shutdown)
	void run();
};

}

#endif //PROJECT_ROS_NGIMU_H
