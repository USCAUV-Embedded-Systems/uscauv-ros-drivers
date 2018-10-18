#include <ros/ros.h>
#include <ros_esccontrol/ESCThrottle.h>
#include <ros_esccontrol.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float64.h>

#include <chrono>
#include <thread>
#include <cmath>

#define QUEUE_SIZE 10

int main(int argc, char** argv)
{
	ros::init(argc, argv, "turtle_motion");
	
	ros::NodeHandle node("turtle_motor_test");
	ros::Publisher throttlePublisher = node.advertise<ros_esccontrol::ESCThrottle>("/esccontrol/esc_throttle", QUEUE_SIZE);
	
	ros_esccontrol::setMotor(M_HORIZ_LEFT, 0, throttlePublisher);
	ros_esccontrol::setMotor(M_HORIZ_RIGHT, 0, throttlePublisher);
	ros_esccontrol::setMotor(M_VERT_FRONTLEFT, 0, throttlePublisher);
	ros_esccontrol::setMotor(M_VERT_FRONTRIGHT, 0, throttlePublisher);
	ros_esccontrol::setMotor(M_VERT_BACKLEFT, 0, throttlePublisher);
	ros_esccontrol::setMotor(M_VERT_BACKRIGHT, 0, throttlePublisher);
		
	ROS_INFO("Starting motor test.  All motors should run forwards (towards the front or top of the robot)");
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	
	ROS_INFO("Testing horiz left...");
	ros_esccontrol::setMotor(M_HORIZ_LEFT, 1, throttlePublisher);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	ros_esccontrol::setMotor(M_HORIZ_LEFT, 0, throttlePublisher);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	
	ROS_INFO("Testing horiz right...");
	ros_esccontrol::setMotor(M_HORIZ_RIGHT, 1, throttlePublisher);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	ros_esccontrol::setMotor(M_HORIZ_RIGHT, 0, throttlePublisher);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	
	ROS_INFO("Testing vert front left...");
	ros_esccontrol::setMotor(M_VERT_FRONTLEFT, 1, throttlePublisher);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	ros_esccontrol::setMotor(M_VERT_FRONTLEFT, 0, throttlePublisher);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	
	ROS_INFO("Testing vert front right...");
	ros_esccontrol::setMotor(M_VERT_FRONTRIGHT, 1, throttlePublisher);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	ros_esccontrol::setMotor(M_VERT_FRONTRIGHT, 0, throttlePublisher);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	
	ROS_INFO("Testing vert back left...");
	ros_esccontrol::setMotor(M_VERT_BACKLEFT, 1, throttlePublisher);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	ros_esccontrol::setMotor(M_VERT_BACKLEFT, 0, throttlePublisher);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	
	ROS_INFO("Testing vert back right...");
	ros_esccontrol::setMotor(M_VERT_BACKRIGHT, 1, throttlePublisher);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	ros_esccontrol::setMotor(M_VERT_BACKRIGHT, 0, throttlePublisher);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}
