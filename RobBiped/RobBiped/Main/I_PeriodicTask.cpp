/*
 * I_PeriodicTask.cpp
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

#include "I_PeriodicTask.h"

void I_PeriodicTask::setExecutionPeriod(execType _timerType, uint16_t _period)
{
	timerType = _timerType;
	executionPeriod = _period;
	switch (timerType) {
		case execType::inMillis:
			lastTimeExecuted = millis();
		break;
		
		case execType::inMicros:
			lastTimeExecuted = micros();
		break;
	}
}

uint16_t I_PeriodicTask::getExecutionPeriod()
{
	return executionPeriod;
}

uint64_t I_PeriodicTask::getLastMillisExecuted()
{
	return lastTimeExecuted;
}

bool I_PeriodicTask::getExecutionFlag()
{
	bool return_val;
	
	switch (timerType) {
		case execType::inMillis:
			currentMillis = millis();
			if (abs(currentMillis - lastTimeExecuted) >= executionPeriod) {
				lastTimeExecuted = currentMillis;
				return_val = true;
			}
			else return_val = false;
			break;
			
		case execType::inMicros:
			currentMicros = micros();
			if (abs(currentMicros - lastTimeExecuted) >= executionPeriod) {
				lastTimeExecuted = currentMicros;
				return_val = true;
			}
			else return_val = false;
			break;
	}
	
	return return_val;
}