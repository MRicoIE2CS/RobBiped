// Joint.h

#ifndef _JOINT_h
#define _JOINT_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include "MG996R.h"


class Joint {
	
	private:
	
	MG996R servo;
	
	double maxAngleAllowed = PI;
	double minAngleAllowed = -PI;
	double calibration_offsetAngle = HALF_PI;
	bool invertDirection = false;
	
	double assignedAngle = 0;
	
	public:
	
	void cleanCalibrationValues();
	void invertAngleSign(bool yes_no);
	void calibration_setMinAngle(bool catchCurrentAngle, double _angle);
	void calibration_setMaxAngle(bool catchCurrentAngle, double _angle);
	void calibration_setZero(bool catchCurrentAngle, double _angle);
	void calibration_ZeroFineAdjust();	//How to do?	
	
	uint16_t getPWMPulseWidthUpdate();
	//unsigned int getPulseWidthApplied();
	bool isUpdateNeeded();
	
	bool setAngleTarget_rad(double _ang);
	double getAssignedAnlge();
	double getZeroOffset();
	
};

#endif

