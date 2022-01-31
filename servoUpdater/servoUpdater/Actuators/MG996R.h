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
	
		uint16_t minPulse = 100;
		uint16_t maxPulse = 510;
		uint8_t maxAngle_deg = 180;
		uint8_t maxAngle_rad = PI;
		
		uint8_t index = 0;
		
		double angleAssigned = 0;
		int pulseWidthAssigned; 
		int pulseWidthApplied;
		
		int angleToPulse(int _ang);
		int angleToPulse(double _ang);
		
		
		double degToRad(int _ang);
		int radToDeg(double _ang);
	
	public:
		
		void setMinPulse(uint16_t _pulse);
		void setMaxPulse(uint16_t _pulse);
		
		int getPulseWidthToSend();
		bool isNewPulseWidth();
		
		bool setAngleTarget(double _ang);
		bool setAngleTarget(int _ang);
	
	};


#endif

