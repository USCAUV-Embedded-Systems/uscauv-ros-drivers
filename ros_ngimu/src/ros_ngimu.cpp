#include "ros_ngimu.h"

#include <ros/ros.h>

#include <asio.hpp>

#include <utility>
#include <functional>
#include <iomanip>

#define SERIAL_PORT "/dev/ttyACM0"
#define QUEUE_SIZE 100
#define NODE_NAME "ngimu"

namespace ros_ngimu {



	void ros_ngimu::onSerialRead(const asio::error_code &errorCode, size_t bytes_transferred) {

		// send byte to osc99
		OscSlipDecoderProcessByte(&oscSlipDecoder, serialBuffer);

		// queue ourselves up for the next byte
		asio::async_read(serialPort, asio::buffer(&serialBuffer, 1),
						 std::bind(&ros_ngimu::onSerialRead, this, std::placeholders::_1, std::placeholders::_2));
	}

	void ros_ngimu::onGetPacket(OscPacket * const oscPacket)
	{
		// set callback for each message to the message function
		oscPacket->processMessage = std::bind(&ros_ngimu::onGetMessage, this, std::placeholders::_1, std::placeholders::_2);

		// try to process the packet
		OscError oscError = OscPacketProcessMessages(oscPacket);
		if (oscError != OscErrorNone) {
			ROS_WARN("OSC packet decode failed: %s", OscErrorGetMessage(oscError));
		}
	}

	void ros_ngimu::onGetMessage(const OscTimeTag * const oscTimeTag, OscMessage * const oscMessage)
	{
		// configure float formatting for log messages
		std::cout.setf(std::ios::fixed, std::ios::floatfield);
		std::cout.setf(std::ios::showpoint);

		if(OscAddressMatch(oscMessage->oscAddressPattern, "/quaternion"))
		{
			// the numbers are floats, but ROS requires doubles.
			// so, we have to use these floats as an intermediary
			float w, x, y, z;

			OscMessageGetArgumentAsFloat32(oscMessage, &w);
			OscMessageGetArgumentAsFloat32(oscMessage, &x);
			OscMessageGetArgumentAsFloat32(oscMessage, &y);
			OscMessageGetArgumentAsFloat32(oscMessage, &z);

			quaternion.quaternion.w = w;
			quaternion.quaternion.x = x;
			quaternion.quaternion.y = y;
			quaternion.quaternion.z = z;

			//std::cout << "Got quaternion: " << quaternion.quaternion.w << ", " << quaternion.quaternion.x << ", " << quaternion.quaternion.y << ", " << quaternion.quaternion.z << std::endl;

			prepareHeader(quaternion.header);
			quaternionPublisher.publish(quaternion);

		}

		else if(OscAddressMatch(oscMessage->oscAddressPattern, "/earth"))
		{
			float x, y, z;

			OscMessageGetArgumentAsFloat32(oscMessage, &x);
			OscMessageGetArgumentAsFloat32(oscMessage, &y);
			OscMessageGetArgumentAsFloat32(oscMessage, &z);

			earthAccel.vector.x = x;
			earthAccel.vector.y = y;
			earthAccel.vector.z = z;

			//std::cout << "Got earth: " << earthAccel.vector.x << ", " << earthAccel.vector.y << ", " << earthAccel.vector.z << std::endl;

			prepareHeader(earthAccel.header);
			accelPublisher.publish(earthAccel);

		}
		else if(OscAddressMatch(oscMessage->oscAddressPattern, "/euler"))
		{
			float x, y, z;

			OscMessageGetArgumentAsFloat32(oscMessage, &x);
			OscMessageGetArgumentAsFloat32(oscMessage, &y);
			OscMessageGetArgumentAsFloat32(oscMessage, &z);

			eulerAngles.vector.x = x;
			eulerAngles.vector.y = y;
			eulerAngles.vector.z = z;

			//std::cout << "Got euler: " << eulerAngles.vector.x << ", " << eulerAngles.vector.y << ", " << eulerAngles.vector.z << std::endl;

			prepareHeader(eulerAngles.header);
			eulerPublisher.publish(eulerAngles);

		}

	}

	void ros_ngimu::prepareHeader(std_msgs::Header & header)
	{
		header.seq++;
		header.stamp = ros::Time::now();
	}


	void ros_ngimu::run() {
		// keep ticking ROS and ASIO
		while (ros::ok()) {
			ros::spinOnce();
			io_service.run_one();
		}
	}

	ros_ngimu::ros_ngimu(std::string const &serialPortPath, ros::NodeHandle & node) :
			io_service(),
			serialPort(io_service, serialPortPath),
			oscSlipDecoder(),
			serialBuffer(),
			eulerPublisher(node.advertise<geometry_msgs::Vector3Stamped>(NODE_NAME "/euler", QUEUE_SIZE)),
			quaternionPublisher(node.advertise<geometry_msgs::QuaternionStamped>(NODE_NAME "/quaternion", QUEUE_SIZE)),
			accelPublisher(node.advertise<geometry_msgs::Vector3Stamped>(NODE_NAME "/earthAccel", QUEUE_SIZE))
	{
		serialPort.set_option(asio::serial_port::baud_rate(115200));

		// Initialise OSC reader
		OscSlipDecoderInitialise(&oscSlipDecoder);
		oscSlipDecoder.processPacket = std::bind(&ros_ngimu::onGetPacket, this, std::placeholders::_1);

		// start the io service reading
		asio::async_read(serialPort, asio::buffer(&serialBuffer, 1),
							   std::bind(&ros_ngimu::onSerialRead, this, std::placeholders::_1, std::placeholders::_2));
	}

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, NODE_NAME);
	ros::NodeHandle node;

	try
	{
		ros_ngimu::ros_ngimu imu(SERIAL_PORT, node);

		ROS_INFO("Driver started...");

		imu.run();
	}
	catch(std::exception & ex)
	{
		ROS_ERROR("Exception in IMU driver: %s", ex.what());
	}

}


