#ifndef ROS_ESCCONTROL_H
#define ROS_ESCCONTROL_H

#include <ros/ros.h>
#include <ros_esccontrol/ESCThrottle.h>


#define M_VERT_FRONTLEFT 3
#define M_VERT_BACKLEFT 2
#define M_VERT_BACKRIGHT 6
#define M_VERT_FRONTRIGHT 5
#define M_HORIZ_LEFT 1
#define M_HORIZ_RIGHT 4

// Convenience header for ros_esccontrol.
namespace ros_esccontrol
{

// Function to send a message to the ESC controller
void setMotor(int num, float throttle, ros::Publisher & publisher)
{
	ESCThrottle throttleMessage;
	throttleMessage.motor_num = num;
	throttleMessage.power = throttle;
	publisher.publish(throttleMessage);		
}
	
}

#endif