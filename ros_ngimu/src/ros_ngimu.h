//
// Created by jamie on 10/24/18.
//

#ifndef ROS_NGIMU_H
#define ROS_NGIMU_H

#include <asio.hpp>
#include <memory>
#include <utility>

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

	// ASIO event handler for characters read
	void onSerialRead(const asio::error_code& errorCode, std::size_t bytes_transferred);

	// decodes the given buffer from SLIP to plain bytes.
	// Returns decoded byte array and its length.
	// On error, prints a ROS error and returns as much of the message as it could
	static std::pair<std::shared_ptr<char>, size_t> SLIPDecode(asio::streambuf * inputBuffer, size_t size);

public:

	// construct with serial port
	ros_ngimu(std::string const & serialPortPath);

	// run event loop (never returns until node is shutdown)
	void run();
};

}

#endif //PROJECT_ROS_NGIMU_H
