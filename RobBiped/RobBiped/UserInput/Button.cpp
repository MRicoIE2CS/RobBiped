/*
 * Button.cpp
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