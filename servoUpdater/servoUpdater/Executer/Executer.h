// Executer.h



#ifndef _EXECUTER_h
#define _EXECUTER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif



#include "I_Task.h"
//#include "../Communications/I2C.h"
#include "../Actuators/UpdateServos.h"
#include "../Actuators/SignalGenerator.h"





class Executer {
	
	private:
	
	SignalGenerator signalGenerator_0;
	SignalGenerator signalGenerator_1;
	UpdateServos servoUpdater;
	
	public:
	
		void init();
		
		void execution();
	
	};

#endif

