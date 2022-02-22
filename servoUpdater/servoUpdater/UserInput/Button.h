// Button.h

#ifndef _BUTTON_h
#define _BUTTON_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

class Button {
	
	private:
		
		friend class UserInput;
		
		uint8_t HWpin;
		bool value;
		
		void setup(uint8_t _pin, uint8_t _mode);
		
		bool readHWvalue();
		
	public:
	
		
		bool getValue();
};

#endif

