/*
 * Potentiometer.h
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
	
		friend class UserInput;
		
		uint8_t HWpin;
		uint16_t value;
		ExpFilter expFilter;
		
		void setup(uint8_t _pin, double _Kfilter);
		
		uint16_t readHWvalue();
		
	public:
		
		
		uint16_t getValue();
	};


#endif

