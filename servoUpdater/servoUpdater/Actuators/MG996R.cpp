// 
// 
// 

#include "MG996R.h"


bool MG996R::setTargetAngle(double _ang){
	if (_ang < 0 || _ang > maxAngle_rad) {
		return true;
	}
	else {
		pulseWidthAssigned = angleToPulse(_ang);
		return false;
	}
	
}

uint16_t MG996R::getPulseWidthAssigned(){
	pulseWidthApplied = pulseWidthAssigned;
	return pulseWidthAssigned;
}

bool MG996R::isNewPulseWidth(){
	bool out = pulseWidthAssigned != pulseWidthApplied;
	return out;
}

uint16_t MG996R::angleToPulse(double _ang){
	uint16_t pulse = round((double)minPulse + _ang * ((double)(maxPulse-minPulse) / maxAngle_rad));
	return pulse;
}

