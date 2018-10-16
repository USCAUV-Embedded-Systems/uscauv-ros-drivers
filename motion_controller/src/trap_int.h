#include <iostream>
//class for z dir error
class TrapezoidalIntegrator {
	/*add a data point where sub is, compare to correct
	val consider elapsed time since last point*/
	void add(double dt, double value);

	/*get integral, returns integral val with
	every data point so far. integral of error*/
	double get() const;

	private:
		double total;
		double prevPoint=0;
}
