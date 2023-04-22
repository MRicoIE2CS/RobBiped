/*
 * MG996R.h
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

#ifndef _MG996R_h
#define _MG996R_h

#include "Arduino.h"

class MG996R {

	private:

		// minPulse and maxPulse are calibrated so that all the range covers 180deg (PI rads) with relative precision
		uint16_t min_pulse_ = 111;	
		uint16_t max_pulse_ = 508;
		double min_angle_rad_ = 0.0;
		double max_angle_rad_ = PI;

		uint16_t pulse_width_assigned_; 
		uint16_t pulse_width_applied_;

		uint16_t angle_to_pulse(double _ang);

	public:

		bool set_target_angle(double _ang);	//Input angle from 0 to PI rads

		uint16_t get_pulse_width_assigned();
		bool is_new_pulse_width();
	};

#endif
