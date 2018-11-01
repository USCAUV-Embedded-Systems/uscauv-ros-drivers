#include "ros_ngimu.h"

#include <ros/ros.h>

#include <asio.hpp>

#include <utility>
#include <functional>
#include <iomanip>

#include <osc/OscReceivedElements.h>
#include <osc/OscPrintReceivedElements.h>

#define SERIAL_PORT "/dev/ttyACM0"
#define QUEUE_SIZE 100
#define NODE_NAME "ngimu"

namespace ros_ngimu {

// SLIP binary constants
	const char SLIP_END = static_cast<const char>(0xC0);
	const char SLIP_ESC = static_cast<const char>(0xDB);
	const char SLIP_ESC_END = static_cast<const char>(0xDC);
	const char SLIP_ESC_ESC = static_cast<const char>(0xDD);


	void ros_ngimu::onSerialRead(const asio::error_code &errorCode, size_t bytes_transferred) {
		// transfer message, up to and including the delimiter, to our char array
		std::pair<std::shared_ptr<char>, size_t> OSCMessage = SLIPDecode(&serialBuffer, bytes_transferred);

		//std::cout << "packet size: " << OSCMessage.second << std::endl;

		// many packets appear to be corrupt and their size is not a multiple of 4
		if(OSCMessage.second % 4 != 0)
		{
			//std::cout << "Skipping corrupt packet..." << std::endl;
		}
		else
		{
			try
			{
				osc::ReceivedPacket oscPacket(OSCMessage.first.get(), OSCMessage.second);

				//std::cout << oscPacket << std::endl;

				if(oscPacket.IsBundle())
				{
					osc::ReceivedBundle bundle(oscPacket);

					for(auto messageIterator = bundle.ElementsBegin(); messageIterator != bundle.ElementsEnd(); ++messageIterator)
					{
						onGetMessage(osc::ReceivedMessage(*messageIterator));
					}
				}
			}
			catch(osc::MalformedBundleException & ex)
			{
				// corrupt data, ignore
			}

		}

		// queue ourselves up for the next packet
		asio::async_read_until(serialPort, serialBuffer, SLIP_END,
							   std::bind(&ros_ngimu::onSerialRead, this, std::placeholders::_1, std::placeholders::_2));
	}

	std::pair<std::shared_ptr<char>, size_t> ros_ngimu::SLIPDecode(asio::streambuf *inputBuffer, size_t size) {
		// note: the decoded messge will have <= size bytes, so we just create an array of size bytes
		std::shared_ptr<char> decodedBytes(new char[size], array_deleter<char>());

		std::istream bufferStream(inputBuffer);

		bool seenEnd = false;
		bool prevCharWasEsc = false;
		size_t nextOutputIndex = 0;
		for (size_t inputIndex = 0; inputIndex < size && !seenEnd; ++inputIndex) {
			char currByte;
			bufferStream >> currByte;
			//printf("%hhx ", currByte);

			if (inputIndex == size - 1) {
				if (currByte != SLIP_END) {
					ROS_ERROR("SLIP packet does not end in a END byte! Last index was: %lu", size);
				}
			}

			switch (currByte) {
				case SLIP_END:
					seenEnd = true;
					prevCharWasEsc = false;

					// don't copy byte
					break;

				case SLIP_ESC:
					prevCharWasEsc = true;
					// don't copy byte
					break;

				case SLIP_ESC_END:
					if (prevCharWasEsc) {
						// write an END
						decodedBytes.get()[nextOutputIndex] = SLIP_END;
						++nextOutputIndex;
					} else {
						// no escape character, write an ESC_END
						decodedBytes.get()[nextOutputIndex] = SLIP_ESC_END;
						++nextOutputIndex;
					}

					prevCharWasEsc = false;
					break;

				case SLIP_ESC_ESC:
					if (prevCharWasEsc) {
						// write an ESC
						decodedBytes.get()[nextOutputIndex] = SLIP_ESC;
						++nextOutputIndex;
					} else {
						// no escape character, write an ESC_ESC
						decodedBytes.get()[nextOutputIndex] = SLIP_ESC_ESC;
						++nextOutputIndex;
					}

					prevCharWasEsc = false;
					break;

				default:
					//plain old bytes

					if (prevCharWasEsc) {
						ROS_ERROR("Unrecognized SLIP escape sequence!");
					}

					decodedBytes.get()[nextOutputIndex] = currByte;
					++nextOutputIndex;

					break;
			}

		}

		return std::make_pair(decodedBytes, nextOutputIndex);
	}

	void ros_ngimu::onGetMessage(osc::ReceivedMessage const & message)
	{
		osc::ReceivedMessage::const_iterator messageIter = message.ArgumentsBegin();

		// configure float formatting for log messages
		std::cout.setf(std::ios::fixed, std::ios::floatfield);
		std::cout.setf(std::ios::showpoint);

		if(message.AddressPattern() == std::string("/quaternion"))
		{
			quaternion.quaternion.w = messageIter->AsFloat();
			++messageIter;
			quaternion.quaternion.x = messageIter->AsFloat();
			++messageIter;
			quaternion.quaternion.y = messageIter->AsFloat();
			++messageIter;
			quaternion.quaternion.z = messageIter->AsFloat();
			++messageIter;

			//std::cout << "Got quaternion: " << quaternion.quaternion.w << ", " << quaternion.quaternion.x << ", " << quaternion.quaternion.y << ", " << quaternion.quaternion.z << std::endl;

			prepareHeader(quaternion.header);
			quaternionPublisher.publish(quaternion);

		}
		else if(message.AddressPattern() == std::string("/earth"))
		{
			earthAccel.vector.x = messageIter->AsFloat();
			++messageIter;
			earthAccel.vector.y = messageIter->AsFloat();
			++messageIter;
			earthAccel.vector.z = messageIter->AsFloat();
			++messageIter;

			//std::cout << "Got earth: " << earthAccel.vector.x << ", " << earthAccel.vector.y << ", " << earthAccel.vector.z << std::endl;

			prepareHeader(earthAccel.header);
			accelPublisher.publish(earthAccel);

		}
		else if(message.AddressPattern() == std::string("/euler"))
		{
			eulerAngles.vector.x = messageIter->AsFloat();
			++messageIter;
			eulerAngles.vector.y = messageIter->AsFloat();
			++messageIter;
			eulerAngles.vector.z = messageIter->AsFloat();
			++messageIter;

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
			serialBuffer(),
			eulerPublisher(node.advertise<geometry_msgs::Vector3Stamped>(NODE_NAME "/euler", QUEUE_SIZE)),
			quaternionPublisher(node.advertise<geometry_msgs::QuaternionStamped>(NODE_NAME "/quaternion", QUEUE_SIZE)),
			accelPublisher(node.advertise<geometry_msgs::Vector3Stamped>(NODE_NAME "/earthAccel", QUEUE_SIZE))
	{
		// start the io service reading
		asio::async_read_until(serialPort, serialBuffer, SLIP_END,
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


