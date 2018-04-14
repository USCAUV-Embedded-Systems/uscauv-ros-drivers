#include "controller_sub.hpp"

#define NUM_ESC 6

// calibration values for each ESC
const static int center_points[] = {1570, 1565, 1570, 1570, 1570, 1500};
const static int8_t inversions[] = {-1, 1, -1, -1, 1, 1};

// "radius" of the PWM signal sent to the motors -- distanct in us from center point to full forward or reverse
#define PWM_RADIUS 500

Controller::Controller(){

	controller = new PCA9685(I2C_BUS, I2C_ADDRESS);
 	controller->setPWMFreq(250);
	sub_esc = node.subscribe("esccontrol/esc_throttle", 100, &Controller::chatterESCThrottle, this);
	ROS_INFO("ESC controller is ready...");
}

void Controller::chatterESCThrottle (const esccontrol_msgs::ESCThrottleConstPtr &esc_state){
	int target_value;
    ROS_INFO("CMD ESC throttle %d",  esc_state->motor_num);

    // ensure power is in correct range
    float power = esc_state->power;
    if(power > 1)
    {
    	power = 1.0;
    }
    else if(power < -1)
    {
    	power = 1.0;
    }

    if(esc_state->motor_num <= 0 || esc_state->motor_num > NUM_ESC)
    {
    	ROS_ERROR("Invalid ESC number %d", esc_state->motor_num);
    }
    else
    {
        int motor_index = esc_state->motor_num;
        int motor_port = 15 - motor_index;
        int us_offset_for_power = power * 300;
        int desired_us = inversions[motor_index] * (us_offset_for_power + center_points[motor_index]);

        controller->setPWM(motor_port, (desired_us / 4000.0) * 4095);
    }
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "controller_sub");
	Controller c;
    ros::spin();
}


