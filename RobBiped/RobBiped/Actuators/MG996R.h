/*
 * MG996R.h
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

#ifndef _MG996R_h
#define _MG996R_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif


class MG996R {
	
	private:
		
		// minPulse and maxPulse are calibrated so that all the range covers 180deg (PI rads) with relative precision
		uint16_t minPulse = 111;	
		uint16_t maxPulse = 508;
		double maxAngle_rad = PI;
		
		uint16_t pulseWidthAssigned; 
		uint16_t pulseWidthApplied;
		
		uint16_t angleToPulse(int16_t _ang);
		uint16_t angleToPulse(double _ang);
	
	public:
		
		bool setTargetAngle(double _targetAngle);	//Input angle from 0 to PI rads
		
		uint16_t getPulseWidthAssigned();
		bool isNewPulseWidth();
	
	};


#endif

