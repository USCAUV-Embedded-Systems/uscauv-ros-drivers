#include <ros/ros.h>

#include <asio.hpp>

#define SERIAL_PORT "/dev/ttyUSB0"

asio::io_service io_service;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "turtle_motion");
	
	asio::serial_port imuSerialPort(io_service, SERIAL_PORT)
	
	// keep ticking ROS and ASIO
	while(ros::ok())
	{
		ros::spinOnce();
		io_service.run_one();
	}
}