// 
// 
// 

#include "Joint.h"


uint16_t Joint::getPWMPulseWidthUpdate(){
	uint16_t pulseWidth = servo.getPulseWidthAssigned();
	return pulseWidth;
}

bool Joint::isUpdateNeeded(){
	bool out = servo.isNewPulseWidth();
	return out;
}

bool Joint::setAngleTarget_rad(double _ang){
	if (_ang < -PI || _ang > PI
	|| (_ang < minAngleAllowed || _ang > maxAngleAllowed)
	) {
		return true;
	}
	else {
		angleAssigned = _ang;
		servo.setTargetAngle(_ang);
		return false;
	}
}