/*
 * SignalGenerator.cpp
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

#include "SignalGenerator.h"


double SignalGenerator::generateTrajectory(){
	
	unsigned long currentMillis = millis();
	
	double nextOutput = offset + amplitude * sin( (double)currentMillis/(double)period_ms + (double)phaseShift) ;
	
	lastTimeExecuted = currentMillis;
	
	return nextOutput;
}

void SignalGenerator::configureSignal(SignalGenerator::SignalType _type, unsigned int _period, unsigned int _amplitude, unsigned int _offset, unsigned int _phaseShift){
	signalType = _type;
	period_ms = _period;
	amplitude = _amplitude;
	offset = _offset;
	phaseShift = _phaseShift;
}

void SignalGenerator::init(){
	
	lastCalculatedTime = millis();
	last_output = 0;
}

void SignalGenerator::update(){
	
}