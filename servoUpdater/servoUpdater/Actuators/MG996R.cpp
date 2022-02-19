// 
// 
// 

#include "MG996R.h"

/*
void MG996R::calibration_setPoint1(double _currentangle){
	
	calibration_Point1.pulseWidth = pulseWidthApplied;
	calibration_Point1.angle_rad = -HALF_PI;//_currentangle;
	Serial.println("CalibrationPoint1: " + (String)pulseWidthApplied);
}

void MG996R::calibration_setPoint2(double _currentangle){
	
	calibration_Point2.pulseWidth = pulseWidthApplied;
	calibration_Point2.angle_rad = HALF_PI;//_currentangle;
	Serial.println("CalibrationPoint2: " + (String)pulseWidthApplied);
}
*/



void MG996R::setTargetAngle(double _ang){
	
	pulseWidthAssigned = angleToPulse(_ang);
}

// bool MG996R::setAngleTarget(int16_t _ang){
// 	if (_ang < 0 || _ang > 180) {
// 		return true;
// 	}
// 	else {
// 		angleAssigned = degToRad(_ang);
// 		pulseWidthAssigned = angleToPulse(_ang);
// 		return false;
// 	}
// }

uint16_t MG996R::getPulseWidthAssigned(){
	pulseWidthApplied = pulseWidthAssigned;
	return pulseWidthAssigned;
}

// uint16_t MG996R::getPulseWidthApplied(){
// 	return pulseWidthApplied;
// }

bool MG996R::isNewPulseWidth(){
	bool out = pulseWidthAssigned != pulseWidthApplied;
	return out;
}

uint16_t MG996R::angleToPulse(int16_t _ang){
	uint16_t pulse = minPulse + _ang * ((maxPulse-minPulse) / maxAngle_deg);
	return pulse;
}

uint16_t MG996R::angleToPulse(double _ang){
	//TODO: Change minPulse and maxPulse for calibrated pulses for each 0 and PI angles
	uint16_t pulse = minPulse + _ang * ((double)(maxPulse-minPulse) / (double)maxAngle_rad);
	return pulse;
}

double MG996R::degToRad(int16_t _ang){
	double ang_rad = (double)(_ang* PI) /180 ;
	return ang_rad;
}

int16_t MG996R::radToDeg(double _ang){
	int16_t ang_deg = _ang /PI * 180;
	return ang_deg;
}