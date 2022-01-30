// 
// 
// 

#include "MG996R.h"



bool MG996R::setAngleTarget(double _ang){
	if (_ang < 0 || _ang > 180) {
		return true;
	}
	else {
		angleAssigned = _ang;
		return false;
	}
}

int MG996R::angleToPulse(double ang){
	int pulse = map((int)ang,0, 180, minPulse, maxPulse);
	return pulse;
}