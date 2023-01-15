// Executor.h



#ifndef _EXECUTOR_h
#define _EXECUTOR_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif



#include "I_PeriodicTask.h"
#include "../Actuators/JointsManager.h"
#include "../Utils/SignalGenerator.h"
#include "Configs.h"
#include "../Utils/ExponentialFilter.h"
#include "../UserInput/UserInput.h"
#include "../Sensors/ForceSensorsManager.h"
#include "../Sensors/GyroscopeAccelerometerManager.h"

class Executor {
	
	private:
		
		
		/////____________ OBJECTS: __//
		Configuration::Configs config;
		ExpFilter pot1Filter;
		///// END OBJECTS: __//
		
		/////____________ OBJECT TASKS: __//
		UserInput userInput;
		SignalGenerator signalGenerator_0;
		SignalGenerator signalGenerator_1;
		JointsManager servoUpdater;
		ForceSensorsManager forceSensorsManager;
		GyroscopeAccelerometerManager gyroscopeAccelerometerManager;
		///// END OBJECT TASKS __//
	
		/////____________ PRIVATE FUNCTIONS: __//
		void setup();
		void associations();
		
		void inputs();
		void mainExecution();
		void outputs();
		///// END PRIVATE FUNCTIONS: __//
	
	public:
		
		/////____________ PUBLIC FUNCTIONS: __//
		void init();
		
		void execution();
	
	};

#endif

