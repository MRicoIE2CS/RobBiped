/*
 * ReadUserInput.cpp
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

#include "UserInput.h"

void UserInput::assoc_GPIO(Configs::UserInputPins &_gpio){
	gpio_ = &_gpio;
	configuration();
}

void UserInput::configuration(){

	forward_button_.setup(gpio_->forward_button, INPUT_PULLDOWN);
	back_button_.setup(gpio_->back_button, INPUT_PULLDOWN);

	potentiometer1_.setup(gpio_->potentiometer1, 0.97);
	potentiometer2_.setup(gpio_->potentiometer2, 0.97);
}

void UserInput::update(){

	potentiometer1_.read_HW_value();
	potentiometer2_.read_HW_value();
	forward_button_.read_HW_value();
	back_button_.read_HW_value();
}

uint16_t UserInput::get_analog_value(AnalogInputList select_input){

	switch (select_input) {
		case AnalogInputList::potentiometer1:
			return potentiometer1_.get_value();
			break;
		case AnalogInputList::potentiometer2:
			return potentiometer2_.get_value();
			break;
	}
}


bool UserInput::get_digital_value(DigitalInputList select_input){

	switch (select_input) {
		case DigitalInputList::forward_button:
			return forward_button_.get_value();
			break;
		case DigitalInputList::back_button:
			return back_button_.get_value();
			break;
	}
}
