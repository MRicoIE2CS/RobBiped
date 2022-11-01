// 
// 
// 

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