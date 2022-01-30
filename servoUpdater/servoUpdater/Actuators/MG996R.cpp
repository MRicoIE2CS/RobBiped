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

int MG996R::angleToPulse(int _ang){
	int pulse = minPulse + (_ang * (maxPulse-minPulse) / maxAngle);
	return pulse;
}

int MG996R::angleToPulse(double _ang){
	double pulse = angleToPulse(radToDeg(_ang));
	return pulse;
}

double MG996R::degToRad(int _ang){
	double ang_rad = (double)(_ang* PI) /180 ;
	return ang_rad;
}

int MG996R::radToDeg(double _ang){
	int ang_deg = _ang /PI * 180;
	return ang_deg;
}