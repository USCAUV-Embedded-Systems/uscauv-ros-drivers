#include <i2c_cpp/I2C.h>

#include <ros/ros.h>

#include <vector>
#include <iostream>
#include <thread>


int main(int argc, char** argv)
{
	I2C i2c(1, 0x40);
	
	ROS_INFO("Checking that si7021 is connected");
	i2c.write_bytes({0xE7});
	uint8_t status_register = i2c.read_bytes(1)[0];
	
	if(status_register == 0x3A)
	{
		ROS_INFO("Connected and verified!");
	}
	else
	{
		ROS_ERROR("Status register reads as %x, (should have been 0x3a), communication error");
		return 1;
	}
	
	i2c.write_bytes({0xF3});
	
	uint16_t rawTemperature = 0;
	
	while(true)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		
		try
		{
			std::vector<uint8_t> temperatureBytes = i2c.read_bytes(2);
			rawTemperature = (temperatureBytes[0] << 8) | temperatureBytes[1];
		}
		catch(I2CException & ex)
		{
			// device not ready yet, retry
		}
	}	
}