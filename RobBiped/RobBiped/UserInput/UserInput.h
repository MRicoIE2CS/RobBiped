/*
 * ReadUserInput.h
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

#ifndef _READUSERINPUT_h
#define _READUSERINPUT_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include "../Main/I_PeriodicTask.h"
#include "../Main/Configs.h"
#include "Potentiometer.h"
#include "Button.h"

using namespace Configuration;

class UserInput : public I_PeriodicTask {
	
	private:
		
		friend class Executor;
		
		Potentiometer potentiometer1;
		Potentiometer potentiometer2;
		Button thinButton1;
		Button thinButton2;
		Button squareButton;
		
		Configs::UserInputPins *gpio;
		
		void assocGPIO(Configs::UserInputPins &_gpio);
		void configuration();
		
		void update();
		
	public:
		
		enum class DigitalInputList { thinButton1, thinButton2, squareButton };
		enum class AnalogInputList { potentiometer1, potentiometer2 };
		
		uint16_t getAnalogValue(AnalogInputList selectInput);
		bool getDigitalValue(DigitalInputList selectInput);
	
	};



#endif

