/*
 * MG996R.cpp
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

#include "MG996R.h"


bool MG996R::setTargetAngle(double _ang){
	if (_ang < 0 || _ang > maxAngle_rad) {
		Serial.println("Joint overlimit! | Angle: " + (String)_ang);
		return true;
	}
	else {
		pulseWidthAssigned = angleToPulse(_ang);
		return false;
	}
	
}

uint16_t MG996R::getPulseWidthAssigned(){
	pulseWidthApplied = pulseWidthAssigned;
	return pulseWidthAssigned;
}

bool MG996R::isNewPulseWidth(){
	bool out = pulseWidthAssigned != pulseWidthApplied;
	return out;
}

uint16_t MG996R::angleToPulse(double _ang){
	uint16_t pulse = round((double)minPulse + _ang * ((double)(maxPulse-minPulse) / maxAngle_rad));
	return pulse;
}

