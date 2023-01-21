/*
 * Potentiometer.cpp
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

#include "Potentiometer.h"


void Potentiometer::setup(uint8_t _pin, double _K_filter){
	
	HW_pin_ = _pin;
	adcAttachPin(HW_pin_);
	exp_filter_.set_exp_constant(_K_filter);
}

uint16_t Potentiometer::read_HW_value(){
	
	value_ = exp_filter_.filter(analogRead(HW_pin_));
}

uint16_t Potentiometer::get_value(){
	
	return value_;
}