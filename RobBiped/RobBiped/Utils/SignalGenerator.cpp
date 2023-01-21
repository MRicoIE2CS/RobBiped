/*
 * SignalGenerator.cpp
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

#include "SignalGenerator.h"


double SignalGenerator::generate_trajectory(){
	
	uint64_t current_millis = millis();
	
	double nextOutput = offset_ + amplitude_ * sin( (double)current_millis/(double)period_ms_ + (double)phase_shift_) ;
	
	return nextOutput;
}

void SignalGenerator::configure_signal(SignalGenerator::SignalType _type, uint16_t _period, uint16_t _amplitude, uint16_t _offset, uint16_t _phase_shift){
	signal_type_ = _type;
	period_ms_ = _period;
	amplitude_ = _amplitude;
	offset_ = _offset;
	phase_shift_ = _phase_shift;
}

void SignalGenerator::init(){
	
	last_calculated_time_ = millis();
	last_output_ = 0;
}
