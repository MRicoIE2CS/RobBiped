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
		
		uint8_t index = 0;
		
		double angleAssigned = 0;
		
		int angleToPulse(double _ang);
		
		double degToRad(int _ang);
		double radToDeg(int _ang);
	
	public:
		
		void setMinPulse(uint16_t _pulse);
		void setMaxPulse(uint16_t _pulse);
		
		int getPulseWidth();
		
		bool setAngleTarget(double _ang);
		bool setAngleTarget(int _ang);
	
	};


#endif

