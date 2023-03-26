/*
 * Executor.h
 *
 * Copyright 2023 Mikel Rico Abajo (https://github.com/MRicoIE2CS)

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

#include "Arduino.h"

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
		Configuration::Configs config_;
		///// END OBJECTS: __//
		
		/////____________ TASK OBJECTS: __//
		UserInput user_input_;
		JointsManager servo_updater_;
		ForceSensorsManager force_sensors_manager_;
		GyroscopeAccelerometerManager gyroscope_accelerometer_manager_;
		///// END OBJECT TASKS __//
	
		/////____________ PRIVATE FUNCTIONS: __//
		void associations();
		
		void inputs();
		void main_execution();
		void outputs();
		///// END PRIVATE FUNCTIONS: __//
	
	public:
		
		/////____________ PUBLIC FUNCTIONS: __//
		void init();
		
		void execution();
		
	};

#endif
