/*
 * ExponentialFilter.h
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

#ifndef _EXPONENTIALFILTER_h
#define _EXPONENTIALFILTER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

//TODO: Change file for Signals
//TODO: New class: Hysteresis: A greater change in value is needed in order to change direction of change, than the case when incrementing in same direction


class ExpFilter {
	
	protected:
		
		double lastFilteredValue = 0;
		double expK = 0.9;
		
	public:
		
		double setExpConstant(double k);
		double filter(double rawValue);
		double filter(uint8_t rawValue);
		double filter(int8_t rawValue);
		double filter(uint16_t rawValue);
		double filter(int16_t rawValue);
		double filter(uint32_t rawValue);
		double filter(int32_t rawValue);
		double filter(uint64_t rawValue);
		double filter(int64_t rawValue);
	};


#endif

