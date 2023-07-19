/*
 * Button.cpp
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

#include "Button.h"


void Button::setup(uint8_t _pin, uint8_t _mode){
	
	HW_pin_ = _pin;
	pinMode(HW_pin_,_mode);
}

bool Button::read_HW_value(){
	
	bool currentValue = digitalRead(HW_pin_);
	uint32_t currentMillis = millis();
	uint32_t timeSinceLastChange = abs((int32_t)value_change_triggger_time_ - (int32_t)currentMillis);
	
	if (value_ != currentValue && (timeSinceLastChange > value_change_delay_ms_)) {
		value_ = currentValue;
		value_change_triggger_time_ = currentMillis;
	}
	
	return value_;
}

bool Button::get_value(){
	
	return value_;
}