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

#include <osc/OscReceivedElements.h>

#include <ros/ros.h>

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
	asio::streambuf serialBuffer;

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

	// decodes the first packet from the given buffer from SLIP to plain bytes.
	// Returns decoded byte array and its length.
	// On error, prints a ROS error and returns as much of the message as it could
	static std::pair<std::shared_ptr<char>, size_t> SLIPDecode(asio::streambuf * inputBuffer, size_t size);

	// called when an OSC message is received from the IMU
	void onGetMessage(osc::ReceivedMessage const & message);

	// Prepare a ROS message header for sending the the next message
	void prepareHeader(std_msgs::Header & header);

public:

	// construct with serial port
	ros_ngimu(std::string const & serialPortPath, ros::NodeHandle & node);

	// run event loop (never returns until node is shutdown)
	void run();
};

}

#endif //PROJECT_ROS_NGIMU_H
