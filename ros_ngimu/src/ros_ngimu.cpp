#include "ros_ngimu.h"

#include <ros/ros.h>

#include <asio.hpp>

#include <utility>
#include <functional>

#define SERIAL_PORT "/dev/ttyUSB0"

namespace ros_ngimu {

// SLIP binary constants
	const char SLIP_END = static_cast<const char>(0xC0);
	const char SLIP_ESC = static_cast<const char>(0xDB);
	const char SLIP_ESC_END = static_cast<const char>(0xDC);
	const char SLIP_ESC_ESC = static_cast<const char>(0xDD);


	void ros_ngimu::onSerialRead(const asio::error_code &errorCode, size_t bytes_transferred) {
		// transfer message, up to and including the delimiter, to our char array
		std::pair<std::shared_ptr<char>, size_t> OSCMessage = SLIPDecode(&serialBuffer, bytes_transferred);

		// queue ourselves up for the next packet
		asio::async_read_until(serialPort, serialBuffer, SLIP_END,
							   std::bind(&ros_ngimu::onSerialRead, this, std::placeholders::_1, std::placeholders::_2));

	}

	std::pair<std::shared_ptr<char>, size_t> ros_ngimu::SLIPDecode(asio::streambuf *inputBuffer, size_t size) {
		// note: the decoded messge will have <= size bytes, so we just create an array of size bytes
		std::shared_ptr<char> decodedBytes(new char[size], array_deleter<char>());

		bool prevCharWasEsc = false;
		size_t nextOutputIndex = 0;
		for (size_t inputIndex = 0; inputIndex < size; ++inputIndex) {
			char currByte;
			std::istream(inputBuffer) >> currByte;

			if (inputIndex == size - 1) {
				if (currByte != SLIP_END) {
					ROS_ERROR("SLIP packet does not end in a END byte!");
				}
			}

			switch (currByte) {
				case SLIP_END:
					if (inputIndex != size - 1) {
						ROS_ERROR("Unexpected END byte in middle of SLIP packet!");
					}

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

			return std::make_pair(decodedBytes, nextOutputIndex);

		}
	}


	void ros_ngimu::run() {
		// keep ticking ROS and ASIO
		while (ros::ok()) {
			ros::spinOnce();
			io_service.run_one();
		}
	}

	ros_ngimu::ros_ngimu(std::string const &serialPortPath) :
			io_service(),
			serialPort(io_service, serialPortPath),
			serialBuffer() {
		// start the io service reading
		asio::async_read_until(serialPort, serialBuffer, SLIP_END,
							   std::bind(&ros_ngimu::onSerialRead, this, std::placeholders::_1, std::placeholders::_2));
	}

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "turtle_motion");

	ros_ngimu::ros_ngimu imu(SERIAL_PORT);
	imu.run();
}


