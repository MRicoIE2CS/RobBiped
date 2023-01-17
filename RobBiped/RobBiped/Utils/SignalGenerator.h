/*
 * SignalGenerator.h
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

#ifndef _SIGNALGENERATOR_h
#define _SIGNALGENERATOR_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include "../Main/I_PeriodicTask.h"


class SignalGenerator : public I_PeriodicTask{
	
	public:
		
		void init();
		
		void update();
		
		double generateTrajectory();
		
		enum class SignalType { sine, triangular, square, saw };
			
		void configureSignal(SignalType _type, uint16_t _period, uint16_t _amplitude, uint16_t _offset, uint16_t _phaseShift);
		
		
	private:
	
		uint64_t lastCalculatedTime;
		
		SignalType signalType = SignalType::sine;
		
		uint16_t period_ms = 500;
		
		double amplitude = 1;
		
		double offset = 0;
		
		uint16_t phaseShift = 0;
		
		double last_output = 0;
		
	};



#endif

