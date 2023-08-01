/*
 * Joint.cpp
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

#include "Joint.h"

uint16_t Joint::get_PWM_pulse_width_update(){
	uint16_t pulse_width = servo_.get_pulse_width_assigned();
	return pulse_width;
}

bool Joint::is_update_needed(){
	bool out = servo_.is_new_pulse_width();
	return out;
}

bool Joint::set_angle_target_rad(double _ang){
	if (invert_direction_) _ang = - _ang;

	if (_ang < -PI || _ang > PI
	|| (!invert_direction_ && (_ang < min_angle_allowed_ || _ang > max_angle_allowed_))
	|| (invert_direction_ && (_ang < max_angle_allowed_ || _ang > min_angle_allowed_))
	)
	{
		if (!invert_direction_ && _ang < min_angle_allowed_
				|| invert_direction_ &&_ang > min_angle_allowed_)
			servo_.set_target_angle(min_angle_allowed_ + calibration_offset_angle_);

		else if (!invert_direction_ && _ang > max_angle_allowed_
				|| invert_direction_ && _ang < max_angle_allowed_)
			servo_.set_target_angle(max_angle_allowed_ + calibration_offset_angle_);

		return false;
	}
	else {
		bool ret_val = servo_.set_target_angle(_ang + calibration_offset_angle_);

		if (!ret_val)
		{
			return false;
		}
		else
		{
			assigned_angle_ = _ang;
			return true;
		}
	}
}

void Joint::clean_calibration_values(){

	max_angle_allowed_ = (invert_direction_) ? -PI : PI;
	min_angle_allowed_ = (invert_direction_) ? PI : -PI;
	calibration_offset_angle_ = HALF_PI;
}

void Joint::calibration_set_min_angle(bool catch_current_angle, double _angle){

	if (catch_current_angle){

		min_angle_allowed_ = assigned_angle_;
	}
	else {

		min_angle_allowed_ = (invert_direction_) ? - _angle : _angle;
	}
}

void Joint::calibration_set_max_angle(bool catch_current_angle, double _angle){

	if (catch_current_angle){

		max_angle_allowed_ = assigned_angle_;
	}
	else {

		max_angle_allowed_ = (invert_direction_) ? - _angle : _angle;
	}
}

void Joint::calibration_set_zero(bool catch_current_angle, double _angle){

	if (catch_current_angle){

		calibration_offset_angle_ = assigned_angle_ + HALF_PI;
	} 
	else {

		calibration_offset_angle_ = ((invert_direction_) ? - _angle : _angle) + HALF_PI;
	}
}

void Joint::invert_angle_sign(bool yes_no){

	invert_direction_ = yes_no;
}

double Joint::get_assigned_anlge(){

	return (invert_direction_) ? - assigned_angle_ : assigned_angle_;
}

double Joint::get_zero_offset(){

	return calibration_offset_angle_ - HALF_PI;
}
