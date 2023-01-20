/*
 * Joint.h
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

#ifndef _JOINT_h
#define _JOINT_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include "MG996R.h"


class Joint {
	
	private:
	
	MG996R servo_;
	
	double max_angle_allowed_ = PI;
	double min_angle_allowed_ = -PI;
	double calibration_offset_angle_ = HALF_PI;
	bool invert_direction_ = false;
	
	double assigned_angle_ = 0;
	
	public:
	
	void clean_calibration_values();
	void invert_angle_sign(bool yes_no);
	void calibration_set_min_angle(bool catch_current_angle, double _angle);
	void calibration_set_max_angle(bool catch_current_angle, double _angle);
	void calibration_set_zero(bool catch_current_angle, double _angle);
	void calibration__zero_fine_adjust();	//How to do?	
	
	uint16_t get_PWM_pulse_width_update();
	//unsigned int getPulseWidthApplied();
	bool is_update_needed();
	
	bool set_angle_target_rad(double _ang);
	double get_assigned_anlge();
	double get_zero_offset();
	
};

#endif

