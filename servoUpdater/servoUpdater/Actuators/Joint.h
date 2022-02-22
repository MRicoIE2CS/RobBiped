// Joint.h

#ifndef _JOINT_h
#define _JOINT_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include "MG996R.h"

//TODO: Joint class includes MG996R servo, all public methods for assigning angle values, calibrating joints...
// MG996R includes the transformation of the angle to PWM pulses 


class Joint {
	
	private:
	
	MG996R servo;
	
	double maxAngleAllowed = PI/2;
	double minAngleAllowed = -PI/2;
	double calibration_offsetAngle = 0;
	bool invertDirection = false;
	
	double angleAssigned = 0;
	
	public:
	
	void invertAngleSign(bool yes_no);
	void calibration_setMinAngle(bool catchCurrentAngle, double _angle);
	void calibration_setMaxAngle(bool catchCurrentAngle, double _angle);
	void calibration_setZero(bool catchCurrentAngle, double _angle);
	void calibration_ZeroFineAdjust();	//How to do?	
	
	uint16_t getPWMPulseWidthUpdate();
	//unsigned int getPulseWidthApplied();
	bool isUpdateNeeded();
	
	bool setAngleTarget_rad(double _ang);
	//bool setAngleTarget_deg(double _ang);
	
};

#endif

