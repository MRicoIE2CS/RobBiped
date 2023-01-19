/*
 * ExponentialFilter.cpp
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

#include "ExponentialFilter.h"


double ExpFilter::set_exp_constant(double k){
	exp_K_ = k;
}

double ExpFilter::filter(double rawValue){
	double filteredValue;
	filteredValue = last_filtered_value_*exp_K_ + (1.0-exp_K_)*rawValue;
	last_filtered_value_ = filteredValue;
	return filteredValue;
}

double ExpFilter::filter(uint8_t rawValue){
	double filteredValue;
	filteredValue = last_filtered_value_*exp_K_ + (1.0-exp_K_)*(double)rawValue;
	last_filtered_value_ = filteredValue;
	return filteredValue;
}

double ExpFilter::filter(int8_t rawValue){
	double filteredValue;
	filteredValue = last_filtered_value_*exp_K_ + (1.0-exp_K_)*(double)rawValue;
	last_filtered_value_ = filteredValue;
	return filteredValue;
}

double ExpFilter::filter(uint16_t rawValue){
	double filteredValue;
	filteredValue = last_filtered_value_*exp_K_ + (1.0-exp_K_)*(double)rawValue;
	last_filtered_value_ = filteredValue;
	return filteredValue;
}

double ExpFilter::filter(int16_t rawValue){
	double filteredValue;
	filteredValue = last_filtered_value_*exp_K_ + (1.0-exp_K_)*(double)rawValue;
	last_filtered_value_ = filteredValue;
	return filteredValue;
}

double ExpFilter::filter(uint32_t rawValue){
	double filteredValue;
	filteredValue = last_filtered_value_*exp_K_ + (1.0-exp_K_)*(double)rawValue;
	last_filtered_value_ = filteredValue;
	return filteredValue;
}

double ExpFilter::filter(int32_t rawValue){
	double filteredValue;
	filteredValue = last_filtered_value_*exp_K_ + (1.0-exp_K_)*(double)rawValue;
	last_filtered_value_ = filteredValue;
	return filteredValue;
}

double ExpFilter::filter(uint64_t rawValue){
	double filteredValue;
	filteredValue = last_filtered_value_*exp_K_ + (1.0-exp_K_)*(double)rawValue;
	last_filtered_value_ = filteredValue;
	return filteredValue;
}

double ExpFilter::filter(int64_t rawValue){
	double filteredValue;
	filteredValue = last_filtered_value_*exp_K_ + (1.0-exp_K_)*(double)rawValue;
	last_filtered_value_ = filteredValue;
	return filteredValue;
}