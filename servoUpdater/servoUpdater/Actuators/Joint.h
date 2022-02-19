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
	
	double maxAngleAllowed = HALF_PI/2;
	double minAngleAllowed = -HALF_PI/2;
	
	double angleAssigned = 0;
	
// 	struct class point {
// 		uint16_t pulseWidth;
// 		double angle_rad;
// 	};
// 	point calibration_Point1;
// 	point calibration_Point2;
	
//	uint8_t calibrationZero_pulse = 310;
	
	
	double degToRad(int _ang);
	int16_t radToDeg(double _ang);
	
	public:
	
	void calibration_setMinAngleNow();
	void calibration_setMaxAngleNow();
	void calibration_setZeroNow();
	void calibration_ZeroFineAdjust();	//How to do?
	void invertAngleSign(bool yes_no);
	
	uint16_t getPWMPulseWidthUpdate();
	//unsigned int getPulseWidthApplied();
	bool isUpdateNeeded();
	
	bool setAngleTarget_rad(double _ang);
	//bool setAngleTarget_deg(double _ang);
	
};

#endif

