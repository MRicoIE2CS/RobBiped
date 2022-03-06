// Potentiometer.h

#ifndef _POTENTIOMETER_h
#define _POTENTIOMETER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include "../Utils/ExponentialFilter.h"

class Potentiometer {
	
	private:
	
		friend class UserInput;
		
		uint8_t HWpin;
		uint16_t value;
		ExpFilter expFilter;
		
		void setup(uint8_t _pin, double _Kfilter);
		
		uint16_t readHWvalue();
		
	public:
		
		
		uint16_t getValue();
	};


#endif

