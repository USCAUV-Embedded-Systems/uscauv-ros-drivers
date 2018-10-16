
#ifndef TURTLE_MOTION_H
#define TURTLE_MOTION_H

#include <ros/ros.h>
#include <ros_esccontrol/ESCThrottle.h>
#include <ros_esccontrol.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>

#include <chrono>
#include <thread>
#include <cmath>

// generated header files for this service
#include <motion_controller/SetDepth.h>

namespace motion_controller
{

class TurtleMotion
{
public:
	ros::NodeHandle node;
	
private:
	
	ros::Subscriber imuSubscriber;
	
	// setpoints
	float desiredRoll;
	float desiredPitch;
	float desiredYaw;
	float desiredDepth;
	
	// last data recieved from sensors
	float sensorRoll;
	float sensorPitch;
	float sensorYaw;
	float sensorDepth;
	
	// outputs from the various control loops
	MotorPowers rollPIDPowers;
	MotorPowers pitchPIDPowers;
	MotorPowers yawPIDPowers;
	MotorPowers depthPIDPowers;
	MotorPowers forwardsMotionPowers;
	
	AnglePID<float> rollPID;
	AnglePID<float> pitchPID;
	AnglePID<float> yawPID;
		
	void chatterNavxEuler(const geometry_msgs::Vector3Stamped &vector);
	
public:

	TurtleMotion::TurtleMotion():
	node("turtle_motion"),
	throttlePublisher(node.advertise<ros_esccontrol::ESCThrottle>("/esccontrol/esc_throttle", QUEUE_SIZE)),
	imuSubscriber(node.subscribe("/navx_micro/euler", QUEUE_SIZE, &TurtleMotion::chatterNavxEuler, this)),
	desiredRoll(0),
	desiredPitch(0),
	desiredYaw(0),
	desiredDepth(0),
	
	{

	}

	// update angle PID loops based on current sensor values
	// called by chatterNavxEuler
	void TurtleMotion::updatePitchPID();
	void TurtleMotion::updateRollPID();
	void TurtleMotion::updateYawPID();

	// update outputs based on current PID values
	void updateOutputs();
	
};


#endif