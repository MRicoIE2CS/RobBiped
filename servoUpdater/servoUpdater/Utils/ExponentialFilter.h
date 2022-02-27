// ExponentialFilter.h

#ifndef _EXPONENTIALFILTER_h
#define _EXPONENTIALFILTER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

//TODO: Change file for Signals
//TODO: New class: Hysteresis: A greater change in value is needed in order to change direction of change, than the case when incrementing in same direction


class ExpFilter {
	
	private:
		
		double lastFilteredValue = 0;
		double expK = 0.9;
		
	public:
		
		double setExpConstant(double k);
		double filter(double rawValue);
		double filter(uint16_t rawValue);
	};


#endif

