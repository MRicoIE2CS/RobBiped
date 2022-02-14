// Executer.h



#ifndef _EXECUTER_h
#define _EXECUTER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif



#include "I_PeriodicTask.h"
//#include "../Communications/I2C.h"
#include "../Actuators/UpdateServos.h"
#include "../Actuators/SignalGenerator.h"
#include "Configs.h"





class Executer {
	
	private:
		
		
		/////____________ OBJECTS: __//
		Configs config;
		/////____________ END OBJECTS: __//
		
		/////____________ OBJECT TASKS: __//
		SignalGenerator signalGenerator_0;
		SignalGenerator signalGenerator_1;
		UpdateServos servoUpdater;
		/////____________ END OBJECT TASKS __//
	
		/////____________ PRIVATE FUNCTIONS: __//
		void setup();
		void IOs_setup();
		/////____________ END PRIVATE FUNCTIONS: __//
	
	public:
		
		/////____________ PUBLIC FUNCTIONS: __//
		void init();
		
		void execution();
	
	};

#endif

