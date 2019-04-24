//
// Created by uscauv on 3/26/19.
//
// This node acts as a bridge between the Teensy microcontroller in the auxilary pod, and
// the ROS system running on the Jetson.
//

#ifndef PROJECT_AUXPOD_H
#define PROJECT_AUXPOD_H

#include <ros/ros.h>

#include <i2c_cpp/I2C.h>

// list of auxpod commands

#define ACMD_HELLO 'h' // causes the device to respond with a fixed value -- sanity check
#define ACMD_TEMP 't' // returns temp read from temp sensor
#define ACMD_PRESSURE 'p' // returns pressure read by pressure sensor
#define ACMD_FIRE_TORPEDO 'f' // causes firing of torpedo
#define ACMD_MARKER 'm' // causes dropping of marker

// note: namespace is basically required because all of the service files are in this namespace
namespace auxpod
{

class AuxPod
{

	I2C i2c;

	// Low level functions:

	/**
	 * Send a command to the device, without expecting any response
	 * @param command
	 */
	void sendCommand(uint8_t command);

	/**
	 * Send a command to the device and return the four bits produced.
	 * @param command
	 * @return
	 */
	uint32_t readValue(uint8_t command);

public:

	AuxPod(uint8_t i2cBus, uint8_t i2cAddress, ros::NodeHandle & node):
	i2c(i2cBus, i2cAddress)
	{
		// make sure the device is actually present
		if(readValue(ACMD_HELLO) != 1976)
		{
			throw std::runtime_error("Aux pod failed sanity check!  Bad response to HELLO command.");
		}
	}

};

}



#endif //PROJECT_AUXPOD_H
