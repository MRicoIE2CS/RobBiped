/*
 * I_PeriodicTask.cpp
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

#include "I_PeriodicTask.h"

void I_PeriodicTask::set_execution_period(execType _timer_type, uint16_t _period)
{
	timer_type_ = _timer_type;
	execution_period_ = _period;
	switch (timer_type_) {
		case execType::inMillis:
			last_time_executed_ = millis();
		break;
		
		case execType::inMicros:
			last_time_executed_ = micros();
		break;
	}
}

uint16_t I_PeriodicTask::get_execution_period()
{
	return execution_period_;
}

uint64_t I_PeriodicTask::get_last_millis_executed()
{
	return last_time_executed_;
}

bool I_PeriodicTask::get_execution_flag()
{
	bool return_val;
	
	switch (timer_type_) {
		case execType::inMillis:
			current_millis_ = millis();
			if (abs((int32_t)current_millis_ - (int32_t)last_time_executed_) >= execution_period_) {
				last_time_executed_ = current_millis_;
				return_val = true;
			}
			else return_val = false;
			break;
			
		case execType::inMicros:
			current_micros_ = micros();
			if (abs((int32_t)current_micros_ - (int32_t)last_time_executed_) >= execution_period_) {
				last_time_executed_ = current_micros_;
				return_val = true;
			}
			else return_val = false;
			break;
	}
	
	return return_val;
}
