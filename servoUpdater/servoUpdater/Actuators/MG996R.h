// MG996R.h

#ifndef _MG996R_h
#define _MG996R_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif


class MG996R {
	
	private:
		
		// minPulse and maxPulse are calibrated so that all the range covers 180deg (PI rads) with relative precision
		uint16_t minPulse = 111;	
		uint16_t maxPulse = 508;
		double maxAngle_rad = PI;
		
		uint16_t pulseWidthAssigned; 
		uint16_t pulseWidthApplied;
		
		uint16_t angleToPulse(int16_t _ang);
		uint16_t angleToPulse(double _ang);
	
	public:
		
		bool setTargetAngle(double _targetAngle);	//Input angle from 0 to PI rads
		
		uint16_t getPulseWidthAssigned();
		bool isNewPulseWidth();
	
	};


#endif

