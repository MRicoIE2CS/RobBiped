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
	
		unsigned int minPulse = 100;
		unsigned int maxPulse = 510;//520
		unsigned short maxAngle_deg = 180;
		unsigned short maxAngle_rad = PI;
		
		unsigned short index = 0;
		
		double angleAssigned = 0;
		unsigned int pulseWidthAssigned; 
		unsigned int pulseWidthApplied;
		
		unsigned int angleToPulse(int _ang);
		unsigned int angleToPulse(double _ang);
		
		
		double degToRad(int _ang);
		int radToDeg(double _ang);
	
	public:
		
		void setMinPulse(unsigned int _pulse);
		void setMaxPulse(unsigned int _pulse);
		
		unsigned int getPulseWidthToSend();
		unsigned int getPulseWidthApplied();
		bool isNewPulseWidth();
		
		bool setAngleTarget(double _ang);
		bool setAngleTarget(int _ang);
	
	};


#endif

