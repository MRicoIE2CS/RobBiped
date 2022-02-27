// ReadUserInput.h

#ifndef _READUSERINPUT_h
#define _READUSERINPUT_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include "../Main/I_PeriodicTask.h"
#include "../Main/Configs.h"
#include "Potentiometer.h"
#include "Button.h"

class UserInput : public I_PeriodicTask {
	
	private:
		
		friend class Executer;
		
		Potentiometer potentiometer1;
		Potentiometer potentiometer2;
		Button thinButton1;
		Button thinButton2;
		Button squareButton;
		
		Configs::GPIO_defs *gpio;
		
		void assocGPIO(Configs::GPIO_defs &_gpio);
		void configuration();
		
		void update();
		
	public:
		
		enum class DigitalInputList { thinButton1, thinButton2, squareButton };
		enum class AnalogInputList { potentiometer1, potentiometer2};
		
		uint16_t getAnalogValue(AnalogInputList selectInput);
		bool getDigitalValue(DigitalInputList selectInput);
	
	};



#endif

