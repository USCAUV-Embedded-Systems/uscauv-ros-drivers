//
// Created by uscauv on 3/26/19.
//

#include "AuxPod.h"

void auxpod::AuxPod::sendCommand(uint8_t command)
{
	try
	{
		i2c.write_bytes({command});
	}
	catch(I2CException & ex)
	{
		ROS_ERROR("Error sending command to aux pod: %s", ex.what());
	}
}

uint32_t auxpod::AuxPod::readValue(uint8_t command)
{
	try
	{
		i2c.write_bytes({command});
	}
	catch(I2CException & ex)
	{
		ROS_ERROR("Error sending command to aux pod: %s", ex.what());
		return 0;
	}

	try
	{
		std::vector<uint8_t> bytes = i2c.read_bytes(4);

		// Arduino is little endian
		uint32_t value;
		value = bytes[0] | bytes[1] << 8 | bytes[2] << 16 | bytes[3] << 24;
		return value;
	}
	catch(I2CException & ex)
	{
		ROS_ERROR("Error receiving bytes from aux pod: %s", ex.what());
		return 0;
	}
}
