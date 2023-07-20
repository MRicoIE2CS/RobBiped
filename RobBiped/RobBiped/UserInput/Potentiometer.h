/*
 * Potentiometer.h
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

#ifndef _POTENTIOMETER_h
#define _POTENTIOMETER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include "../Utils/ExponentialFilter.h"

class Potentiometer {
	
	private:
		
		uint8_t HW_pin_;
		uint16_t value_;
		ExpFilter exp_filter_;
		
	public:
	
		void setup(uint8_t _pin, uint32_t _time_filter);
	
		uint16_t read_HW_value();
		
		uint16_t get_value();
	};


#endif

