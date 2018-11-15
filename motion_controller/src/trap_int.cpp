#include <iostream>
#include "trap_int.h"

void TrapezoidalIntegrator::add(double dt, double value) {
	total+=(prevPoint+value)*dt/2;
	prevPoint=value;
}

double TrapezoidalIntegrator::get() const {
	return total;
}
