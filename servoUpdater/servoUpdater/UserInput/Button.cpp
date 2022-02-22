// 
// 
// 

#include "Button.h"


void Button::setup(uint8_t _pin, uint8_t _mode){
	
	HWpin = _pin;
	pinMode(HWpin,_mode);
}

bool Button::readHWvalue(){
	
	value = digitalRead(HWpin);
	return value;
}

bool Button::getValue(){
	
	return value;
}