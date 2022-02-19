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
	
		uint16_t minPulse = 100; //110 is 0 calibrated
		uint16_t maxPulse = 510;//513 is PI calibrated
		uint8_t maxAngle_deg = 180;
		uint8_t maxAngle_rad = PI;
		
// 		struct class point {
// 			uint16_t pulseWidth;
// 			double angle_rad;
// 			};
// 		point calibration_Point1;
// 		point calibration_Point2;
		
//		uint8_t calibrationZero_pulse = 310;
		
		//double angleAssigned = 0;
		uint16_t pulseWidthAssigned; 
		uint16_t pulseWidthApplied;
		
		uint16_t angleToPulse(int16_t _ang);
		uint16_t angleToPulse(double _ang);
		
		
		double degToRad(int16_t _ang);
		int16_t radToDeg(double _ang);
	
	public:
		
		
		//void calibration_setPoint1(double _currentangle);
		//void calibration_setPoint2(double _currentangle);
		
		void setTargetAngle(double _targetAngle);
		
		uint16_t getPulseWidthAssigned();
		//unsigned int getPulseWidthApplied();
		bool isNewPulseWidth();
		
		
	
	};


#endif

