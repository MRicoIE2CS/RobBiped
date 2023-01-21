/*
 * ExponentialFilterWithPeakRejection.h
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

#ifndef _EXPONENTIALFILTERWITHPEAKREJECTION_h
#define _EXPONENTIALFILTERWITHPEAKREJECTION_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "ExponentialFilter.h"


class ExpFilterPeakReject: public ExpFilter {
	private:
		double threshold_value_;
		
	public:
		void set_threshold_value(double value);
		double filter_pr(double raw_value, bool accept_step = false);
		double filter_pr(uint8_t raw_value, bool accept_step = false);
		double filter_pr(int8_t raw_value, bool accept_step = false);
		double filter_pr(uint16_t raw_value, bool accept_step = false);
		double filter_pr(int16_t raw_value, bool accept_step = false);
		double filter_pr(uint32_t raw_value, bool accept_step = false);
		double filter_pr(int32_t raw_value, bool accept_step = false);
		double filter_pr(uint64_t raw_value, bool accept_step = false);
		double filter_pr(int64_t raw_value, bool accept_step = false);
	};

#endif
