/*
 * ReadUserInput.cpp
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

#include "UserInput.h"

void UserInput::assoc_GPIO(Configs::UserInputPins &_gpio){
	gpio = &_gpio;
	configuration();
}

void UserInput::configuration(){
	
	square_button_.setup(gpio->squareButton, INPUT_PULLDOWN);
	thin_button1_.setup(gpio->thinButton1, INPUT_PULLDOWN);
	thin_button2_.setup(gpio->thinButton2, INPUT_PULLDOWN);
	
	potentiometer1_.setup(gpio->potentiometer1, 0.97);
	potentiometer2_.setup(gpio->potentiometer2, 0.97);
}

void UserInput::update(){
	
	potentiometer1_.read_HW_value();
	potentiometer2_.read_HW_value();
	square_button_.read_HW_value();
	thin_button1_.read_HW_value();
	thin_button2_.read_HW_value();
}

uint16_t UserInput::get_analog_value(AnalogInputList selectInput){
	
	switch (selectInput) {
		case AnalogInputList::potentiometer1:
			return potentiometer1_.get_value();
			break;
		case AnalogInputList::potentiometer2:
			return potentiometer2_.get_value();
			break;
	}
}


bool UserInput::get_digital_value(DigitalInputList selectInput){
	
	switch (selectInput) {
		case DigitalInputList::squareButton:
			return square_button_.get_value();
			break;
		case DigitalInputList::thinButton1:
			return thin_button1_.get_value();
			break;
		case DigitalInputList::thinButton2:
			return thin_button2_.get_value();
			break;
	}
}