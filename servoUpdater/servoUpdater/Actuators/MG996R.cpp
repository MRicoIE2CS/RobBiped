// 
// 
// 

#include "MG996R.h"



bool MG996R::setAngleTarget(double _ang){
	if (_ang < 0 || _ang > PI) {
		return true;
	}
	else {
		angleAssigned = _ang;
		return false;
	}
}

bool MG996R::setAngleTarget(int _ang){
	if (_ang < 0 || _ang > 180) {
		return true;
	}
	else {
		angleAssigned = degToRad(_ang);
		return false;
	}
}

int MG996R::getPulseWidth(){
	int pulseWidth = angleToPulse(angleAssigned);
	return pulseWidth;
}

int MG996R::angleToPulse(double _ang){
	int pulse = map((int)_ang,0, 180, minPulse, maxPulse);
	return pulse;
}

double MG996R::degToRad(int _ang){
	double ang_rad = _ang /180 * PI;
	return ang_rad;
}

double MG996R::radToDeg(double _ang){
	double ang_deg = _ang /PI * 180;
	return ang_deg;
}