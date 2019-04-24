#include "controller_sub.hpp"

#define NUM_ESC 6

// calibration values for each ESC
const static int center_points[] = {1570, 1565, 1570, 1570, 1570, 1570};
const static int8_t inversions[] = {-1, 1, -1, -1, 1, -1};

// "radius" of the PWM signal sent to the motors -- distanct in us from center point to full forward or reverse
#define PWM_RADIUS 500

Controller::Controller(){

	controller = new PCA9685(I2C_BUS, I2C_ADDRESS);
 	controller->setPWMFreq(250);
	
	sub_esc = node.subscribe("esccontrol/esc_throttle", 100, &Controller::chatterESCThrottle, this);
	ROS_INFO("ESC controller is ready...");
}

void Controller::chatterESCThrottle (const ros_esccontrol::ESCThrottleConstPtr &esc_state){
	int target_value;
    ROS_INFO("CMD ESC throttle: motor %d -> %.03f",  esc_state->motor_num, esc_state->power);

    // ensure power is in correct range
    float power = esc_state->power;
    if(power > 1)
    {
    	power = 1.0;
    }
    else if(power < -1)
    {
    	power = -1.0;
    }

    if(esc_state->motor_num <= 0 || esc_state->motor_num > NUM_ESC)
    {
    	ROS_ERROR("Invalid ESC number %d", esc_state->motor_num);
    }
    else
    {
        int motor_index = esc_state->motor_num - 1;
        int motor_port = esc_state->motor_num;
        int us_offset_for_power = power * 200; 
		//ROS_INFO("us_offset_for_power: %d", us_offset_for_power);
		//ROS_INFO("inversions[motor_index]: %d", inversions[motor_index]);
		//ROS_INFO("center_points[motor_index]: %d", center_points[motor_index]);
		
        int desired_us = (inversions[motor_index] * us_offset_for_power) + center_points[motor_index];
		int pwm_time = static_cast<int>((desired_us / 4000.0) * 4095);
		
		ROS_INFO("Setting PWM %d to %d", motor_port, pwm_time);
        controller->setPWM(motor_port, pwm_time);
    }
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "escccontrol");
	Controller c;
    ros::spin();
}


