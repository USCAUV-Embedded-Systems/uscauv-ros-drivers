#include <ros/ros.h>
#include <esccontrol_msgs/ESCThrottle.h>
#include <ros_esccontrol.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float64.h>

#include <chrono>
#include <thread>
#include <cmath>

// generated header file for this service
#include <uscturtle_motion/GoStraight.h>

#define QUEUE_SIZE 10

namespace uscturtle_motion
{

class TurtleMotion
{
public:
	ros::NodeHandle node;
private:
	
	// publishers and subscribers
	ros::Publisher throttlePublisher;

	ros::Publisher straightDriveSetpointPublisher;
	ros::Publisher straightDriveStatePublisher;
	
	ros::Subscriber imuSubscriber;
	
	volatile float imuYaw = 0;
	volatile double pidCorrection = 0;
	
	std_msgs::Float64 makeFloatMessage(double data)
	{
		std_msgs::Float64 msg;
		msg.data = data;
		return msg;
	}
	
	void chatterNavxEuler(const geometry_msgs::Vector3Stamped &vector)
	{
		//ROS_INFO("Got NavX chatter: %.02f", vector.vector.z);
		imuYaw = 180-vector.vector.z;
	}
	
	void chatterPIDCorrection(const std_msgs::Float64 &correction)
	{
		//ROS_ERROR("Got PID chatter: %.02f", correction.data);
		pidCorrection = correction.data;
	}

public:
	TurtleMotion():
	node("turtle_motion"),
	throttlePublisher(node.advertise<esccontrol_msgs::ESCThrottle>("/esccontrol/esc_throttle", QUEUE_SIZE)),
	straightDriveSetpointPublisher(node.advertise<std_msgs::Float64>("/straight_drive_pid/setpoint", QUEUE_SIZE)),
	straightDriveStatePublisher(node.advertise<std_msgs::Float64>("/straight_drive_pid/state", QUEUE_SIZE)),
	imuSubscriber(node.subscribe("/navx_micro/euler", QUEUE_SIZE, &TurtleMotion::chatterNavxEuler, this))
	{
		//ros_esccontrol::setMotor(M_HORIZ_LEFT, .5, throttlePublisher);
		//ros_esccontrol::setMotor(M_HORIZ_RIGHT, .5, throttlePublisher);
		//std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		//ros_esccontrol::setMotor(M_HORIZ_LEFT, 0, throttlePublisher);
		//ros_esccontrol::setMotor(M_HORIZ_RIGHT, 0, throttlePublisher);
	}
	
	
	bool goStraight(GoStraightRequest & request, GoStraightResponse & response)
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
		
		ROS_INFO("Got initial IMU data, waiting...", request.time);
		
		ros_esccontrol::setMotor(M_HORIZ_LEFT, 0, throttlePublisher);
		ros_esccontrol::setMotor(M_HORIZ_RIGHT, 0, throttlePublisher);
		ros_esccontrol::setMotor(M_VERT_FRONTLEFT, 0, throttlePublisher);
		ros_esccontrol::setMotor(M_VERT_FRONTRIGHT, 0, throttlePublisher);
		ros_esccontrol::setMotor(M_VERT_BACKLEFT, 0, throttlePublisher);
		ros_esccontrol::setMotor(M_VERT_BACKRIGHT, 0, throttlePublisher);
		//std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		//ros_esccontrol::setMotor(M_VERT_BACKLEFT, -.15, throttlePublisher);
		//ros_esccontrol::setMotor(M_VERT_BACKRIGHT, -.15, throttlePublisher);
		std::this_thread::sleep_for(std::chrono::milliseconds(20000));
		
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
		ros::Time hoverTime = diveTime + ros::Duration(.750);
		
		const float initialPower = .5;
		float currentBasePower = 0;
		
		
		float integral = 0;
		float prevError = 0;
		
		const float kP = .05;
		const float kI = .00015;
		const float kD = .04;
		
		bool doneStarting = false;
		bool doneDiving = false;
		
		ROS_INFO("target, current, error, P, I, D, correction, leftPower, rightPower");
		
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
			
			float currentAngle = imuYaw;
			
			// calculate angluar error
			float error;

			if(currentAngle > 270.0 && initialAngle < 90.0)
			{
				error = (currentAngle - 360) - initialAngle;
			}
			else if(currentAngle < 90.0 && initialAngle > 270.0)
			{
				error = currentAngle - (initialAngle - 360.0);
			}
			else
			{
				error = currentAngle - initialAngle;
			}
						
			integral += error;
			float P = kP * error;
			float I = kI * integral;
			float D = -1 * kD * (error-prevError);
			float pidCorrection = P + I + D;
			float leftPower = currentBasePower + pidCorrection;
			float rightPower = currentBasePower - pidCorrection;
			
			prevError = error;
			
			// if we turn to the left, the error will be positive, so in the case of a positive correction, make the left motors
			// spin faster and the right motors spin slower.
			ros_esccontrol::setMotor(M_HORIZ_LEFT, leftPower, throttlePublisher);
			ros_esccontrol::setMotor(M_HORIZ_RIGHT, rightPower, throttlePublisher);
			
			ROS_INFO("%.04f, %.04f, %.04f, %.04f, %.04f, %.04f, %.04f, %.04f, %.04f",
				initialAngle, currentAngle, error, P, I, D, pidCorrection, leftPower, rightPower);
			
			ros::spinOnce();
		}
		
		ros_esccontrol::setMotor(M_HORIZ_LEFT, 0, throttlePublisher);
		ros_esccontrol::setMotor(M_HORIZ_RIGHT, 0, throttlePublisher);
		ros_esccontrol::setMotor(M_VERT_FRONTLEFT, 0, throttlePublisher);
		ros_esccontrol::setMotor(M_VERT_FRONTRIGHT, 0, throttlePublisher);
		ros_esccontrol::setMotor(M_VERT_BACKLEFT, 0, throttlePublisher);
		ros_esccontrol::setMotor(M_VERT_BACKRIGHT, 0, throttlePublisher);
		
		ros_esccontrol::setMotor(M_HORIZ_RIGHT, .75, throttlePublisher);
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		ros_esccontrol::setMotor(M_HORIZ_RIGHT, 0, throttlePublisher);
		
	}
};

}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "turtle_motion");
		
	uscturtle_motion::TurtleMotion turtleMotion;
	
	//turtleMotion.node.advertiseService<uscturtle_motion::TurtleMotion, uscturtle_motion::GoStraightRequest, uscturtle_motion::GoStraightResponse>("GoStraight", &uscturtle_motion::TurtleMotion::goStraight, &turtleMotion);
	
	uscturtle_motion::GoStraightRequest request;
	request.time = 90;
	uscturtle_motion::GoStraightResponse response;
	turtleMotion.goStraight(request, response);
	
	ros::shutdown();
	

}