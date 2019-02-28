
#include "angle_pid.h"
#include "turtle_motion.h"


namespace motion_controller
{


void TurtleMotion::updateAngularPIDLoops()
{
	if(enabled)
	{
		float rollCorrection = rollPID.update(sensorRoll);
		rollPIDPowers.setPower(M_VERT_FRONTRIGHT, -1 * rollCorrection);
		rollPIDPowers.setPower(M_VERT_BACKRIGHT, -1 * rollCorrection);
		rollPIDPowers.setPower(M_VERT_FRONTLEFT, rollCorrection);
		rollPIDPowers.setPower(M_VERT_BACKLEFT, rollCorrection);


		float pitchCorrection = pitchPID.update(sensorPitch);
		pitchPIDPowers.setPower(M_VERT_FRONTRIGHT, pitchCorrection);
		pitchPIDPowers.setPower(M_VERT_FRONTLEFT, pitchCorrection);
		pitchPIDPowers.setPower(M_VERT_BACKRIGHT,-1*pitchCorrection);
		pitchPIDPowers.setPower(M_VERT_BACKLEFT,-1*pitchCorrection);

		float yawCorrection = yawPID.update(sensorYaw);
		yawPIDPowers.setPower(M_HORIZ_RIGHT, -yawCorrection);
		yawPIDPowers.setPower(M_HORIZ_LEFT, yawCorrection);

		updateOutputs();
	}
}

void TurtleMotion::chatterIMUEuler(const geometry_msgs::Vector3Stamped::ConstPtr &vector)
{
	//ROS_INFO("Got IMU data!");

	sensorRoll = static_cast<float>(vector->vector.y);
	sensorPitch = static_cast<float>(vector->vector.x);
	sensorYaw = static_cast<float>(vector->vector.z);

	updateAngularPIDLoops();
}

void TurtleMotion::chatterEcho(const std_msgs::Int16 &data) 
{
	ROS_INFO("Echo sensor value: %d",data);
}

// setpoint services
bool TurtleMotion::setDepth(motion_controller::SetDepthRequest & request, motion_controller::SetDepthResponse & response)
{
	desiredDepth = request.depth;

	ROS_INFO("setDepth() to %.02f m", desiredDepth);

	// TODO: depth PID

	return true;
}

bool TurtleMotion::setForwardsPower(motion_controller::SetForwardsPowerRequest & request, motion_controller::SetForwardsPowerResponse & response)
{
	forwardsMotionPowers.setPower(M_HORIZ_LEFT, request.forwardsPower);
	forwardsMotionPowers.setPower(M_HORIZ_RIGHT, request.forwardsPower);

	return true;
}

bool TurtleMotion::setRollPitchAngles(motion_controller::SetRollPitchAnglesRequest & request, motion_controller::SetRollPitchAnglesResponse & response)
{
	desiredRoll = request.roll;
	desiredPitch = request.pitch;

	rollPID.setTarget(desiredRoll);
	pitchPID.setTarget(desiredPitch);

	updateAngularPIDLoops();

	return true;
}
bool TurtleMotion::setYawAngle(motion_controller::SetYawAngleRequest & request, motion_controller::SetYawAngleResponse & response)
{
	desiredYaw = request.yaw;
	yawPID.setTarget(desiredYaw);

	updateAngularPIDLoops();

	return true;
}
bool TurtleMotion::zero(motion_controller::ZeroRequest & request, motion_controller::ZeroResponse & response)
{
	desiredYaw = sensorYaw;
	desiredPitch = sensorPitch;
	desiredRoll = sensorRoll;

	yawPID.setTarget(desiredYaw);
	pitchPID.setTarget(desiredPitch);
	rollPID.setTarget(desiredRoll);

	ROS_INFO("Set angular targets to (%.02f, %.02f, %.02f)", desiredRoll, desiredPitch, desiredYaw);

	updateAngularPIDLoops();

	return true;
}
bool TurtleMotion::setEnabled(motion_controller::SetEnabledRequest & request, motion_controller::SetEnabledResponse & response)
{
	enabled = request.enabled;

	updateOutputs();

	return true;
}

// update outputs based on current PID values
void TurtleMotion::updateOutputs()
{
	MotorPowers sumMotorPowers;

	// temp hover code
	MotorPowers hoverPowers;
	//hoverPowers.setPower(M_VERT_FRONTLEFT, -.3f);
	//hoverPowers.setPower(M_VERT_FRONTRIGHT, -.09f);
	//hoverPowers.setPower(M_VERT_BACKLEFT, -.25f);
	//hoverPowers.setPower(M_VERT_BACKRIGHT, -.21f);

	if(enabled)
	{
		// set motor outputs by adding each PID loop's outputs together.

		sumMotorPowers = rollPIDPowers + pitchPIDPowers + yawPIDPowers + depthPIDPowers + forwardsMotionPowers + hoverPowers;
	}

	for(int motorNum = 1; motorNum <= NUM_MOTORS; ++motorNum)
	{
		ros_esccontrol::setMotor(motorNum, sumMotorPowers.getPower(motorNum), throttlePublisher);
	}

}

bool TurtleMotion::initMotors(motion_controller::InitMotorsRequest &request,
							  motion_controller::InitMotorsResponse &response)
{
	// initialize and zero motor controllers
	ROS_INFO("Intitializing motor controllers...");
	ros_esccontrol::setMotor(M_HORIZ_LEFT, 0, throttlePublisher);
	ros_esccontrol::setMotor(M_HORIZ_RIGHT, 0, throttlePublisher);
	ros_esccontrol::setMotor(M_VERT_FRONTLEFT, 0, throttlePublisher);
	ros_esccontrol::setMotor(M_VERT_FRONTRIGHT, 0, throttlePublisher);
	ros_esccontrol::setMotor(M_VERT_BACKLEFT, 0,throttlePublisher);
	ros_esccontrol::setMotor(M_VERT_BACKRIGHT, 0, throttlePublisher);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	ROS_INFO("Ready.");

	return true;
}


}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "motion_controller");
		
	motion_controller::TurtleMotion turtleMotion;

	ROS_INFO("Motion controller started!");

	ros::spin();
}