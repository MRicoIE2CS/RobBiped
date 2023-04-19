/*
 * MG996R.cpp
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

#include "MG996R.h"

bool MG996R::set_target_angle(double _ang){
	if (_ang < 0 || _ang > max_angle_rad_) {
		Serial.println("Joint overlimit! | Angle: " + (String)_ang);
		return true;
	}
	else {
		pulse_width_assigned_ = angle_to_pulse(_ang);
		return false;
	}
}

uint16_t MG996R::get_pulse_width_assigned(){
	pulse_width_applied_ = pulse_width_assigned_;
	return pulse_width_assigned_;
}

bool MG996R::is_new_pulse_width(){
	bool out = pulse_width_assigned_ != pulse_width_applied_;
	return out;
}

uint16_t MG996R::angle_to_pulse(double _ang){
	uint16_t pulse = round((double)min_pulse_ + _ang * ((double)(max_pulse_-min_pulse_) / max_angle_rad_));
	return pulse;
}
