/*
 * Executor.h
 *
 * Copyright 2023 Mikel Rico Abajo (MRicoIE2C)

 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at

 * http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

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
		///// END OBJECTS: __//
		
		/////____________ TASK OBJECTS: __//
		UserInput userInput;
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

