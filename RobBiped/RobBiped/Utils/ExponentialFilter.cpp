/*
 * ExponentialFilter.cpp
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

#include "ExponentialFilter.h"

bool ExpFilter::set_time_constant(uint32_t _t_ms){
	if (_t_ms == 0) return false;
	time_constant_ms_ = _t_ms;
	return true;
}

double ExpFilter::filter(double raw_value){
	uint32_t current_millis = millis();
	uint32_t time_interval = current_millis - last_time_executed;

	exp_K_ = exp(-(double)time_interval / (double)time_constant_ms_);

	double filteredValue;
	filteredValue = last_filtered_value_*exp_K_ + (1.0-exp_K_)*raw_value;
	last_filtered_value_ = filteredValue;

	last_time_executed = current_millis;
	return filteredValue;
}

double ExpFilter::filter(uint8_t raw_value){
	return filter((double)raw_value);
}

double ExpFilter::filter(int8_t raw_value){
	return filter((double)raw_value);
}

double ExpFilter::filter(uint16_t raw_value){
	return filter((double)raw_value);
}

double ExpFilter::filter(int16_t raw_value){
	return filter((double)raw_value);
}

double ExpFilter::filter(uint32_t raw_value){
	return filter((double)raw_value);
}

double ExpFilter::filter(int32_t raw_value){
	return filter((double)raw_value);
}

double ExpFilter::filter(uint64_t raw_value){
	return filter((double)raw_value);
}

double ExpFilter::filter(int64_t raw_value){
	return filter((double)raw_value);
}
