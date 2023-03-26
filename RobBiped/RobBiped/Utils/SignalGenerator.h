/*
 * SignalGenerator.h
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

#ifndef _SIGNALGENERATOR_h
#define _SIGNALGENERATOR_h

#include "Arduino.h"

#include "../Main/I_PeriodicTask.h"

class SignalGenerator : public I_PeriodicTask{
	
	public:
		
		void init();
		
		double generate_trajectory();
		
		enum class SignalType { sine, triangular, square, saw };
			
		void configure_signal(SignalType _type, uint16_t _period, uint16_t _amplitude, uint16_t _offset, uint16_t _phase_shift);
		
	private:
	
		uint64_t last_calculated_time_;
		
		SignalType signal_type_ = SignalType::sine;
		
		uint16_t period_ms_ = 500;
		
		double amplitude_ = 1;
		
		double offset_ = 0;
		
		uint16_t phase_shift_ = 0;
		
		double last_output_ = 0;
	};

#endif
