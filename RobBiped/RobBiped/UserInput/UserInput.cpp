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

void UserInput::assocGPIO(Configs::UserInputPins &_gpio){
	gpio = &_gpio;
	configuration();
}

void UserInput::configuration(){
	
	squareButton.setup(gpio->squareButton, INPUT_PULLDOWN);
	thinButton1.setup(gpio->thinButton1, INPUT_PULLDOWN);
	thinButton2.setup(gpio->thinButton2, INPUT_PULLDOWN);
	
	potentiometer1.setup(gpio->potentiometer1, 0.97);
	potentiometer2.setup(gpio->potentiometer2, 0.97);
}

void UserInput::update(){
	
	potentiometer1.readHWvalue();
	potentiometer2.readHWvalue();
	squareButton.readHWvalue();
	thinButton1.readHWvalue();
	thinButton2.readHWvalue();
}

uint16_t UserInput::getAnalogValue(AnalogInputList selectInput){
	
	switch (selectInput) {
		case AnalogInputList::potentiometer1:
			return potentiometer1.getValue();
			break;
		case AnalogInputList::potentiometer2:
			return potentiometer2.getValue();
			break;
	}
}


bool UserInput::getDigitalValue(DigitalInputList selectInput){
	
	switch (selectInput) {
		case DigitalInputList::squareButton:
			return squareButton.getValue();
			break;
		case DigitalInputList::thinButton1:
			return thinButton1.getValue();
			break;
		case DigitalInputList::thinButton2:
			return thinButton2.getValue();
			break;
	}
}