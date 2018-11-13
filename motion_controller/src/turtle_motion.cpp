
#include "angle_pid.h"
#include "turtle_motion.h"


namespace motion_controller
{

		
void TurtleMotion::chatterIMUEuler(const geometry_msgs::Vector3Stamped &vector)
{
	sensorRoll = static_cast<float>(vector.vector.x);
	sensorPitch = static_cast<float>(vector.vector.y);
	sensorYaw = static_cast<float>(180 - vector.vector.z);

	if(enabled)
	{
		rollPID.update(sensorRoll);
		pitchPID.update(sensorPitch);
		yawPID.update(sensorYaw);

		updateOutputs();
	}
}

/*


bool goStraight(GoStraightRequest & request, GoStraightResponse & response, float diveTimeSec)
{
	ros::Rate updateRate(50);
	ros::Rate callbackWaitRate(25);
			
	
	ROS_INFO("Going straight for %.02f seconds", request.time);
	ros::spinOnce();	
			
	// set PID initial state
	float initialAngle = imuYaw;
	straightDriveSetpointPublisher.publish(makeFloatMessage(initialAngle));
	
	ROS_INFO("Initial angle is %.02f degrees", initialAngle);
	
	ros::Duration runDuration(request.time);
	ros::Time endTime = ros::Time::now() + runDuration;
	ros::Time startTime = ros::Time::now();
	ros::Time diveTime = ros::Time::now() + ros::Duration(.50);
	ros::Time hoverTime = diveTime + ros::Duration(diveTimeSec);
	
	const float initialPower = .5;
	float currentBasePower = 0;
	
	AnglePID<float> pidCalculator(false, .05, .0075, .0008, initialAngle, .5);
	
	bool doneStarting = false;
	bool doneDiving = false;

	
	while(endTime > ros::Time::now())
	{
		if(!doneStarting)
		{
			if(ros::Time::now() > diveTime)
			{
				//ROS_INFO("Diving...");
				ros_esccontrol::setMotor(M_VERT_FRONTLEFT, -.5, throttlePublisher);
				ros_esccontrol::setMotor(M_VERT_FRONTRIGHT, -.5, throttlePublisher);
				ros_esccontrol::setMotor(M_VERT_BACKLEFT, -.5, throttlePublisher);
				ros_esccontrol::setMotor(M_VERT_BACKRIGHT, -.5, throttlePublisher);
				doneStarting = true;

			}
			
		}
		else if(!doneDiving)
		{
			if(ros::Time::now() > hoverTime)
			{
				//ROS_INFO("Hovering...");
				ros_esccontrol::setMotor(M_VERT_FRONTLEFT, -.15, throttlePublisher);
				ros_esccontrol::setMotor(M_VERT_FRONTRIGHT, -.09, throttlePublisher);
				ros_esccontrol::setMotor(M_VERT_BACKLEFT, -.21, throttlePublisher);
				ros_esccontrol::setMotor(M_VERT_BACKRIGHT, -.21, throttlePublisher);
				currentBasePower = initialPower;
				doneDiving = true;

			}
		}
		
		updateRate.sleep();
					
		float pidCorrection = pidCalculator.update(imuYaw);
		float leftPower = currentBasePower + pidCorrection;
		float rightPower = currentBasePower - pidCorrection;
					
		// if we turn to the left, the error will be positive, so in the case of a positive correction, make the left motors
		// spin faster and the right motors spin slower.
		ros_esccontrol::setMotor(M_HORIZ_LEFT, leftPower, throttlePublisher);
		ros_esccontrol::setMotor(M_HORIZ_RIGHT, rightPower, throttlePublisher);
		
		ros::spinOnce();
	}
	
	ros_esccontrol::setMotor(M_HORIZ_LEFT, 0, throttlePublisher);
	ros_esccontrol::setMotor(M_HORIZ_RIGHT, 0, throttlePublisher);
	ros_esccontrol::setMotor(M_VERT_FRONTLEFT, 0, throttlePublisher);
	ros_esccontrol::setMotor(M_VERT_FRONTRIGHT, 0, throttlePublisher);
	ros_esccontrol::setMotor(M_VERT_BACKLEFT, 0, throttlePublisher);
	ros_esccontrol::setMotor(M_VERT_BACKRIGHT, 0, throttlePublisher);

	return true;
}

// Turn the given amount of degrees
bool turn(float degrees)
{
	ros::Rate updateRate(50);
	ros::Rate callbackWaitRate(25);
	
	// allow time for the first IMU callback to be delivered
	while(std::abs(imuYaw) < .00001)
	{
		ROS_WARN("Waiting for IMU data");
		callbackWaitRate.sleep();
		ros::spinOnce();
	}
	
	ROS_INFO("Got initial IMU data, waiting...");
	
	
	ROS_INFO("Arcing left");
	ros::spinOnce();	
			
	// set PID initial state
	float initialAngle = imuYaw;
	
	ROS_INFO("Initial angle is %.02f degrees", initialAngle);
	
	// hover!
	ros_esccontrol::setMotor(M_VERT_FRONTLEFT, -.15, throttlePublisher);
	ros_esccontrol::setMotor(M_VERT_FRONTRIGHT, -.09, throttlePublisher);
	ros_esccontrol::setMotor(M_VERT_BACKLEFT, -.22, throttlePublisher);
	ros_esccontrol::setMotor(M_VERT_BACKRIGHT, -.22, throttlePublisher);
			
	AnglePID<float> pidCalculator(true, .06, 0.0, .001, initialAngle + degrees, 2);

	// wait for 5 consecutive good values
	while(pidCalculator.getCyclesUnderThreshold() < 50)
	{	
		updateRate.sleep();
					
		float pidCorrection = pidCalculator.update(imuYaw);
		
		if(pidCorrection > .5)
		{
			pidCorrection = .5;
		}
		else if(pidCorrection < -.5)
		{
			pidCorrection = -.5;
		}
		
		float leftPower = pidCorrection;
		float rightPower = -1 * pidCorrection;
		
					
		// if we turn to the left, the error will be positive, so in the case of a positive correction, make the left motors
		// spin faster and the right motors spin slower.
		ros_esccontrol::setMotor(M_HORIZ_LEFT, leftPower, throttlePublisher);
		ros_esccontrol::setMotor(M_HORIZ_RIGHT, rightPower, throttlePublisher);  
		
		ros::spinOnce();
	}
	

	
}

 */

// update outputs based on current PID values
void TurtleMotion::updateOutputs()
{
	MotorPowers sumMotorPowers;

	if(enabled)
	{
		// set motor outputs by adding each PID loop's outputs together.
		MotorPowers sumMotorPowers = rollPIDPowers + pitchPIDPowers + yawPIDPowers + depthPIDPowers + forwardsMotionPowers;
	}

	for(int motorNum = 1; motorNum <= NUM_MOTORS; ++motorNum)
	{
		ros_esccontrol::setMotor(motorNum, sumMotorPowers.getPower(motorNum), throttlePublisher);
	}
}


}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "turtle_motion");
		
	motion_controller::TurtleMotion turtleMotion;
	
	// initialize and zero motor controllers
	ROS_INFO("Intitializing motor controllers...");
	ros_esccontrol::setMotor(M_HORIZ_LEFT, 0, turtleMotion.throttlePublisher);
	ros_esccontrol::setMotor(M_HORIZ_RIGHT, 0, turtleMotion.throttlePublisher);
	ros_esccontrol::setMotor(M_VERT_FRONTLEFT, 0, turtleMotion.throttlePublisher);
	ros_esccontrol::setMotor(M_VERT_FRONTRIGHT, 0, turtleMotion.throttlePublisher);
	ros_esccontrol::setMotor(M_VERT_BACKLEFT, 0, turtleMotion.throttlePublisher);
	ros_esccontrol::setMotor(M_VERT_BACKRIGHT, 0, turtleMotion.throttlePublisher);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	ROS_INFO("Ready.");
	
	ros::spin();
}