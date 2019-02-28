
#ifndef TURTLE_MOTION_H
#define TURTLE_MOTION_H

#include <ros/ros.h>
#include <ros_esccontrol/ESCThrottle.h>
#include <ros_esccontrol.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <chrono>
#include <thread>
#include <cmath>

#include "motor_powers.h"
#include "angle_pid.h"


// generated header files for this service
#include <motion_controller/SetDepth.h>
#include <motion_controller/SetForwardsPower.h>
#include <motion_controller/SetRollPitchAngles.h>
#include <motion_controller/SetYawAngle.h>
#include <motion_controller/Zero.h>
#include <motion_controller/InitMotors.h>
#include <motion_controller/SetEnabled.h>

#define QUEUE_SIZE 100


namespace motion_controller {

    class TurtleMotion {
    public:
        ros::NodeHandle node;

        ros::Publisher throttlePublisher;

    private:

        ros::Subscriber imuSubscriber;
        ros::Subscriber echoSubscriber;

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

        ros::ServiceServer srv_setRollPitchAngles;
        ros::ServiceServer srv_setDepth;
        ros::ServiceServer srv_setEnabled;
        ros::ServiceServer srv_setForwardsPower;
        ros::ServiceServer srv_setYawAngle;
        ros::ServiceServer srv_zero;
        ros::ServiceServer srv_initMotors;

        // if false, all outputs are disabled, and no data gets fed into PID controllers
        bool enabled;

		void updateAngularPIDLoops();

		void chatterIMUEuler(const geometry_msgs::Vector3Stamped::ConstPtr &vector);
        void chatterEcho(const std_msgs::Int16 &msg);
        // setpoint services
        bool setDepth(motion_controller::SetDepthRequest & request, motion_controller::SetDepthResponse & response);

        bool setForwardsPower(motion_controller::SetForwardsPowerRequest & request, motion_controller::SetForwardsPowerResponse & response);

		bool setRollPitchAngles(motion_controller::SetRollPitchAnglesRequest & request, motion_controller::SetRollPitchAnglesResponse & response);

		bool setYawAngle(motion_controller::SetYawAngleRequest & request, motion_controller::SetYawAngleResponse & response);

		bool zero(motion_controller::ZeroRequest & request, motion_controller::ZeroResponse & response);

		bool setEnabled(motion_controller::SetEnabledRequest & request, motion_controller::SetEnabledResponse & response);

		bool initMotors(motion_controller::InitMotorsRequest &request, motion_controller::InitMotorsResponse &response);

    public:

        TurtleMotion() :
                node("motion_controller"),
                throttlePublisher(node.advertise<ros_esccontrol::ESCThrottle>("/esccontrol/esc_throttle", QUEUE_SIZE)),
                imuSubscriber(node.subscribe("/ngimu/euler", QUEUE_SIZE, &TurtleMotion::chatterIMUEuler, this)),
                echoSubscriber(node.subscribe("br_echo", QUEUE_SIZE, &TurtleMotion::chatterEcho,this)),
                desiredRoll(0),
                desiredPitch(0),
                desiredYaw(0),
                desiredDepth(0),
                sensorRoll(0),
                sensorYaw(0),
                sensorPitch(0),
				sensorDepth(0),
                rollPIDPowers(),
                pitchPIDPowers(),
                yawPIDPowers(),
                depthPIDPowers(),
                forwardsMotionPowers(),
                rollPID(false, .025, 0, 0, 0, 0), //edited Jan 24th
                pitchPID(false, .025, 0, 0, 0, 0),//CHANGE THIS DURRING WET TEST!!!!!
                yawPID(true, .06, /*0.0075*/0, .001, 0, 2),
                /*yawPID(false, 0, 0, 0, 0, 0),*/
                enabled(false),
				srv_setRollPitchAngles(node.advertiseService("SetRollPitchAngles", &TurtleMotion::setRollPitchAngles, this)),
				srv_setDepth(node.advertiseService("SetDepth", &TurtleMotion::setDepth, this)),
				srv_setEnabled(node.advertiseService("SetEnabled", &TurtleMotion::setEnabled, this)),
				srv_setForwardsPower(node.advertiseService("SetForwardsPower", &TurtleMotion::setForwardsPower, this)),
				srv_setYawAngle(node.advertiseService("SetYawAngle", &TurtleMotion::setYawAngle, this)),
				srv_zero(node.advertiseService("Zero", &TurtleMotion::zero, this)),
				srv_initMotors(node.advertiseService("InitMotors", &TurtleMotion::initMotors, this))
		{

        }


        // update outputs based on current PID values
        void updateOutputs();

    };

}


#endif