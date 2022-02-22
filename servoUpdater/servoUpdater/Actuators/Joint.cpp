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
		angleAssigned = _ang + HALF_PI;		// Offset due to difference between joint's coordinate frame and servo's
		
		servo.setTargetAngle(angleAssigned);
		return false;
	}
}

void Joint::calibration_setMinAngle(bool catchCurrentAngle, double _angle){
	
	if (catchCurrentAngle){
		
		minAngleAllowed = angleAssigned;
	}
	else {
		
		minAngleAllowed = _angle;
	}
}

void Joint::calibration_setMaxAngle(bool catchCurrentAngle, double _angle){
	
	if (catchCurrentAngle){
		
		maxAngleAllowed = angleAssigned;
	}
	else {
		
		maxAngleAllowed = _angle;
	}
}

void Joint::calibration_setZero(bool catchCurrentAngle, double _angle){
	
	if (catchCurrentAngle){
		
		calibration_offsetAngle = angleAssigned;
	} 
	else {
		
		calibration_offsetAngle = _angle;
	}
}

void Joint::calibration_ZeroFineAdjust(){
	
	// Useful?
}

void Joint::invertAngleSign(bool yes_no){
	
	invertDirection = yes_no;
}


