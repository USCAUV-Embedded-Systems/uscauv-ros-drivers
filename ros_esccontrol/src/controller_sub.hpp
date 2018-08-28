
#ifndef CONTROLLER_SUB_HPP_
#define CONTROLLER_SUB_HPP_

#define I2C_BUS 1
#define I2C_ADDRESS 0x55


#include <ros/ros.h>
#include <ros_esccontrol/ESCThrottle.h>

#include "PCA9685.h"

class Controller {
	public:
		Controller();

	private:

		ros::NodeHandle node;
		PCA9685 *controller;

		ros::Subscriber sub_esc;

		void chatterESCThrottle (const ros_esccontrol::ESCThrottleConstPtr &esc_state);

};

#endif /* CONTROLLER_SUB_HPP_ */
