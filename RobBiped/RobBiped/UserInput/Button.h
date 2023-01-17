/*
 * Button.h
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

#ifndef _BUTTON_h
#define _BUTTON_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

class Button {
	
	private:
		
		friend class UserInput;
		
		uint8_t HWpin;
		bool value;
		uint32_t valueChangeTrigggerTime;
		uint32_t valueChangeDelay_ms = 250;
		
		void setup(uint8_t _pin, uint8_t _mode);
		
		bool readHWvalue();
		
	public:
	
		
		bool getValue();
};

#endif

