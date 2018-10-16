#ifndef ANGLE_PID_H
#define ANGLE_PID_H

#include <chrono>
#include <thread>
#include <cmath>
#include <iostream>
#include "trap_int.h"

// Class that performs angular PID on a system.
// Is templated so it can be used with either floats or doubles.
// NOTE: Angles input into the class don't have to be normalized

template <typename FPType> 
class AnglePID: public TrapezoidalIntegrator
{
	// controls whether value data will be printed, and whether PID constants
	// will be read from an environment variable
	bool _debug;
	
	FPType _kP;
	FPType _kI;
	FPType _kD;
	
	uint32_t _numCyclesUnderThreshold;

	// if an update has error are below this value, numCyclesUnderThreshold is incremented
	const FPType _completeThreshold;
	
	// guaranteed to already be normalized
	FPType _target;
		
	// data from previous iterations
	FPType _prevError;
	
	// time of previous update
	std::chrono::steady_clock::time_point _prevTimestamp;
	
	// force angle to be between 0 and 360
	static FPType normalizeAngle(FPType origAngle)
	{
		while(origAngle > 360.0)
		{
			origAngle -= 360.0;
		}
		while(origAngle < 0.0)
		{
			origAngle += 360.0;
		}
		
		return origAngle;
	}
	
public: 
	AnglePID(bool debug, FPType kP, FPType kI, FPType kD, FPType target, FPType completeThreshold):
	_debug(debug),
	_kP(kP),
	_kI(kI),
	_kD(kD),
	_numCyclesUnderThreshold(0),
	_completeThreshold(completeThreshold),
	_target(normalizeAngle(target)),
	_prevError(0),
	_prevTimestamp(std::chrono::steady_clock::now())
	{
		// override PID constants from environment variables if they exist
		if(getenv("DEBUG_KP") != nullptr)
		{
			std::istringstream kpStream(std::string(getenv("DEBUG_KP")));
			kpStream >> _kP;
		}
		if(getenv("DEBUG_KI") != nullptr)
		{
			std::istringstream kpStream(std::string(getenv("DEBUG_KI")));
			kpStream >> _kI;
		}
		if(getenv("DEBUG_KD") != nullptr)
		{
			std::istringstream kpStream(std::string(getenv("DEBUG_KD")));
			kpStream >> _kD;
		}
		
		if(_debug)
		{
			// print CSV output column headers
			ROS_INFO("target, current, error, P, I, D, correction");
		}
		
	}
	
	void setTarget(FPType newTarget)
	{
		_target = normalizeAngle(newTarget);
	}
	
	// returns the number of PID update cycles that the actual value
	// has been under the threshold valud here.
	uint32_t getCyclesUnderThreshold() const
	{
		return _numCyclesUnderThreshold;
	}
	
	// update using the current value and return the new output adjustment.
	// Detects time between calls and adjusts the math appropriate for me
	FPType update(FPType current)
	{
		current = normalizeAngle(current);
		
		// get delta time in integer microseconds, then convert to floating point seconds
		FPType deltaTime = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - _prevTimestamp).count() / 1e6;
		_prevTimestamp = std::chrono::steady_clock::now();
		
		// calculate angluar error
		FPType error;
		if(current > 270.0 && _target < 90.0)
		{
			error = (current - 360) - _target;
		}
		else if(current < 90.0 && _target > 270.0)
		{
			error = current - (_target - 360.0);
		}
		else
		{
			error = current - _target;
		}
		
		// update integral
		add(deltaTime, error);
		
		// calculate PID values
		FPType P = _kP * error;
		FPType I = _kI * get();
		FPType D = -1 * _kD * (error-_prevError) / deltaTime;
		
		FPType correction = P + I + D;
		
		if(_debug)
		{
			ROS_INFO("%.04f, %.04f, %.04f, %.04f, %.04f, %.04f, %.04f", _target, current, error, P, I, D, correction);
		}
		
		// check for value under threshold
		if(abs(error) < _completeThreshold)
		{
			++_numCyclesUnderThreshold;
		}
		else
		{
			_numCyclesUnderThreshold = 0;
		}
		
		_prevError = error;
		
		return correction;
	}
};

#endif