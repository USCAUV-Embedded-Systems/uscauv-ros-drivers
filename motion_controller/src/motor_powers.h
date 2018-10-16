#ifndef MOTOR_POWERS_H
#define MOTOR_POWERS_H

#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <iomanip>

// simple struct for representing the set of 6 motor powers
// for the motors on the robot.
class MotorPowers
{
private:
	float motorPowers[NUM_MOTORS] = {0, 0, 0, 0, 0, 0};
	
public:
	
	// empty constructor
	MotorPowers(){}
	
	// default assignment and copy constructor
	MotorPowers & operator=(const MotorPowers & other) = default;
	MotorPowers(const MotorPowers & other) = default;
	
	// get and set motor powers.
	// NOTE: 'num' is 1-indexed!
	void setPower(int num, float power)
	{
		assert(num > 0);
		assert(num <= NUM_MOTORS);
		
		motorPowers[num - 1] = power;
	}
	float getPower(int num)
	{
		assert(num > 0);
		assert(num <= NUM_MOTORS);
		
		return motorPowers[num - 1];
	}
	
	MotorPowers operator+(const MotorPowers & other) const
	{
		MotorPowers newPowers;
		
		for(int index = 0; index < NUM_MOTORS; ++index)
		{
			newPowers.setPower(index + 1, motorPowers[index] + other.getPower(index + 1));
		}
		
		return newPowers;
	}
};

// function to print out motor powers to an ostream
std::ostream operator<<(std::ostream & stream, MotorPowers const & motorPowers)
{
	std::stringstream output;
	output << std::setprecision(3) << std::setw(4);
	output << '[';
	output << "H_L:" << motorPowers.getPower(M_HORIZ_LEFT);
	output << ", H_R:" << motorPowers.getPower(M_HORIZ_RIGHT);
	output << ", V_FL:" << motorPowers.getPower(M_VERT_FRONTLEFT);
	output << ", V_BL:" << motorPowers.getPower(M_VERT_BACKLEFT);
	output << ", V_BR:" << motorPowers.getPower(M_VERT_BACKRIGHT);
	output << ", V_FR:" << motorPowers.getPower(M_VERT_FRONTRIGHT);
	output << ']';
	stream << output.str();
}

#endif