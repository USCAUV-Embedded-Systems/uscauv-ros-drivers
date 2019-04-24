//
// Created by uscauv on 3/26/19.
//

#include "AuxPod.h"

#include <ros/ros.h>

// parameters
#define I2C_BUS 1
#define I2C_ADDRESS 0x10
#define QUEUE_SIZE 100
#define NODE_NAME "auxpod"


int main(int argc, char** argv)
{
	ros::init(argc, argv, NODE_NAME);
	ros::NodeHandle node;

	try
	{
		auxpod::AuxPod pod(I2C_BUS, I2C_ADDRESS, node);

		ROS_INFO("Driver started...");

	}
	catch(std::exception & ex)
	{
		ROS_ERROR("Exception in aux pod driver: %s", ex.what());
	}

}
