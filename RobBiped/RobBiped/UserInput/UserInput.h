/*
 * ReadUserInput.h
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

#ifndef _READUSERINPUT_h
#define _READUSERINPUT_h

#include "Arduino.h"

#include "../Main/I_PeriodicTask.h"
#include "../Main/Configs.h"
#include "Potentiometer.h"
#include "Button.h"

using namespace Configuration;

class UserInput : public I_PeriodicTask {

	private:

		Potentiometer potentiometer1_;
		Potentiometer potentiometer2_;
		Button forward_button_;
		Button back_button_;

		Configs::UserInputPins *gpio_;

	public:

		enum class DigitalInputList { forward_button, back_button };
		enum class AnalogInputList { potentiometer1, potentiometer2 };

		// 0-4095 for 12 bits on 0-3.3V. [ESP32]
		// TODO: Offer the measurement in double, in the range [0-1]
		uint16_t get_analog_value(AnalogInputList select_input);

		bool get_digital_value(DigitalInputList select_input);

		void update();

		void assoc_GPIO(Configs::UserInputPins &_gpio);
		void configuration();

	};

#endif
