/*
 * I_PeriodicTask.h
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

#ifndef _TASK_h
#define _TASK_h

#include "Arduino.h"

class I_PeriodicTask{

	public:

		enum class execType { inMillis, inMicros };
		void set_execution_period(execType _timer_type, uint16_t _period);

		bool get_execution_flag();

		void update();

		// This flag can be used to notify the rest of the tasks that it has been updated.
		// For instance, a sensor could notify the rest of the tasks that it has taken a new measurement.
		bool has_been_updated = false;

		// This flag could be used to control when a task has tried to update, but it was not yet ready (for instance,
		//  a measurement was not ready to be taken), so that it could try it again on the next execution loop.
		bool not_ready_last_time = false;

		// This flag could be used by other tasks to force-trigger the update of this task.
		bool should_be_updated = false;

	protected:

		uint16_t get_execution_period();
		uint64_t get_last_millis_executed();

		execType timer_type_ = execType::inMillis;
		uint16_t execution_period_;
		uint64_t last_time_executed_;

		uint64_t current_millis_;
		uint64_t current_micros_;
	};

#endif
