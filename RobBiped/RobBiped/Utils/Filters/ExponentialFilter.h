/*
 * ExponentialFilter.h
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

#ifndef _EXPONENTIALFILTER_h
#define _EXPONENTIALFILTER_h

#include "Arduino.h"

// Exponential Filter
// Moving Average Filter
// Infinite Impulse Response Filter (IIR)
class ExpFilter {

	protected:

		double last_filtered_value_ = 0;
		uint32_t last_time_executed = millis();

		// exp_K = 1 - e ^ (- delta_t / tau) ; with tau being the time constant.
		double exp_K_;

		// The time constant is the amount of time for the smoothed response of a unit step function to reach
		// 1 - 1/e proportion of the original signal, which approximates to 63.2%.
		uint32_t time_constant_ms_ = 20;

	public:

		/*
		*  @fn void set_time_constant(uint32_t _t_ms)
		*  @brief Sets the time constant of the filter.
		*  The time constant is the amount of time for the smoothed response of a unit step function to reach
		*  1 - 1/e proportion of the original signal, which approximates to 63.2%.
		*
		*  @param[in] _t_ms Time constant in ms.
		*/
		bool set_time_constant(uint32_t _t_ms);

		/*
		*  @fn double get_filtered_value()
		*  @brief Returns the filtered value.
		*/
		double get_filtered_value();

		double filter(double raw_value);
		double filter(uint8_t raw_value);
		double filter(int8_t raw_value);
		double filter(uint16_t raw_value);
		double filter(int16_t raw_value);
		double filter(uint32_t raw_value);
		double filter(int32_t raw_value);
		double filter(uint64_t raw_value);
		double filter(int64_t raw_value);
	};

#endif
