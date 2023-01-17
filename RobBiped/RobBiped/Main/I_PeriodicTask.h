/*
 * I_PeriodicTask.h
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

#ifndef _TASK_h
#define _TASK_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

class I_PeriodicTask{

	public:

		enum class execType { inMillis, inMicros };
		void setExecutionPeriod(execType _timerType, uint16_t _period);

		bool getExecutionFlag();

		void update();

	protected:

		uint16_t getExecutionPeriod();
		uint64_t getLastMillisExecuted();

		execType timerType = execType::inMillis;
		uint16_t executionPeriod;
		uint64_t lastTimeExecuted;

		uint64_t currentMillis;
		uint64_t currentMicros;

	};

#endif

