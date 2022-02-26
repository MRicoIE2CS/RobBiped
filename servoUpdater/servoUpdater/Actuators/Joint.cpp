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
		assignedAngle = _ang + calibration_offsetAngle /*+ HALF_PI*/;		// HALF_PI offset due to difference between joint's coordinate frame and servo's
		
		servo.setTargetAngle(assignedAngle);
		return false;
	}
}

void Joint::cleanCalibrationValues(){
	
	maxAngleAllowed = PI;
	minAngleAllowed = -PI;
	calibration_offsetAngle = 0;
	//invertDirection = false;
}

void Joint::calibration_setMinAngle(bool catchCurrentAngle, double _angle){
	
	if (catchCurrentAngle){
		
		minAngleAllowed = assignedAngle;
	}
	else {
		
		minAngleAllowed = _angle;
	}
}

void Joint::calibration_setMaxAngle(bool catchCurrentAngle, double _angle){
	
	if (catchCurrentAngle){
		
		maxAngleAllowed = assignedAngle;
	}
	else {
		
		maxAngleAllowed = _angle;
	}
}

void Joint::calibration_setZero(bool catchCurrentAngle, double _angle){
	
	if (catchCurrentAngle){
		
		calibration_offsetAngle = assignedAngle;
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

double Joint::getAssignedAnlge(){
	
	return assignedAngle;
}

double Joint::getZeroOffset(){
	
	return calibration_offsetAngle;
}