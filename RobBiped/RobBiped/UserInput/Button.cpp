// 
// 
// 

#include "Button.h"


void Button::setup(uint8_t _pin, uint8_t _mode){
	
	HWpin = _pin;
	pinMode(HWpin,_mode);
}

bool Button::readHWvalue(){
	
	bool currentValue = digitalRead(HWpin);
	uint32_t currentMillis = millis();
	uint32_t timeSinceLastChange = abs(valueChangeTrigggerTime - currentMillis);
	
	if (value != currentValue && (timeSinceLastChange > valueChangeDelay_ms)) {
		value = currentValue;
		valueChangeTrigggerTime = currentMillis;
	}
	
	return value;
}

bool Button::getValue(){
	
	return value;
}