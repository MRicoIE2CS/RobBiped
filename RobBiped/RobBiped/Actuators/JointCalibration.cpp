/*
 * JointsConfig.cpp
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

#include "JointsManager.h"

void JointsManager::calibration_set_angle_to_servo(uint16_t potentiometer_val){

	PCA9685_1_servo_map_[calibration_data_.selected_servo].set_angle_target_rad(calibration_get_angle_from_potentiometer(potentiometer_val));
}

double JointsManager::calibration_get_angle_from_potentiometer(uint16_t potentiometer_val){

	double reading_angle_0;
	if (potentiometer_val < 1000){
		reading_angle_0 = 0;
	}
	else if (potentiometer_val > 3000){
		reading_angle_0 = PI;
	}
	else {
		reading_angle_0 = (double)(potentiometer_val - 1000) / (double)2000 * PI;
	}
	double nextAngle_0 = reading_angle_0 - HALF_PI;

	return nextAngle_0;
}

void JointsManager::calibration_mode_enter_exit_conditions(uint32_t current_millis, bool& sel_button_pressed, bool forward_button_pressed, bool back_button_pressed){

	bool conditionsToEnterCalibration = (current_state_ == State::running) && (forward_button_pressed && back_button_pressed);
	bool conditionsToExitCalibration = (current_state_ == State::calibrating) && (calibration_data_.calibration_state == CalibrationState::servoSelection) && (forward_button_pressed && back_button_pressed);
	if (conditionsToEnterCalibration && calibration_data_.calibration_state_button_change_flag == false) {
		current_state_ = State::calibrating;
		calibration_data_.calibration_state = CalibrationState::servoSelection;
		calibration_data_.calibration_state_button_change_flag = true;
		calibration_data_.last_millis_changed_calibration_state = current_millis;
	}
	else if (conditionsToExitCalibration && calibration_data_.calibration_state_button_change_flag == false){
		current_state_ = State::running;
		calibration_data_.calibration_state_button_change_flag = true;
		calibration_data_.last_millis_changed_calibration_state = current_millis;
	}
}

void JointsManager::calibration_button_pressed_flag_mechanism(uint32_t current_millis){

	if (calibration_data_.calibration_state_button_change_flag
	&& (abs((int32_t)current_millis - (int32_t)calibration_data_.last_millis_changed_calibration_state) > calibration_data_.value_change_delay_ms)
	){
		calibration_data_.calibration_state_button_change_flag = false;
	}
}

void JointsManager::calibration_state_machine(uint32_t current_millis, bool& sel_button_pressed, bool forward_button_pressed, bool back_button_pressed){

	calibration_serial_print(current_millis);

	if (calibration_data_.calibration_state_button_change_flag == false){
		
		if (sel_button_pressed || forward_button_pressed || back_button_pressed){
			calibration_data_.calibration_state_button_change_flag = true;
			calibration_data_.last_millis_changed_calibration_state = current_millis;
		}

		switch (calibration_data_.calibration_state) {
			case CalibrationState::servoSelection:
			calibration_servo_selection(sel_button_pressed, forward_button_pressed, back_button_pressed);
			break;
			case CalibrationState::zeroCalibration:
			calibration_zero_calibration(sel_button_pressed, forward_button_pressed, back_button_pressed);
			break;
			case CalibrationState::firstPointCalibration:
			calibration_first_point_calibration(sel_button_pressed, forward_button_pressed, back_button_pressed);
			break;
			case CalibrationState::secondPointCalibration:
			calibration_second_point_calibration(sel_button_pressed, forward_button_pressed, back_button_pressed);
			break;
		}
	}
}

void JointsManager::calibration_serial_print(uint32_t current_millis){

	if (abs((int32_t)current_millis - (int32_t)calibration_data_.serial_print_last_millis) > calibration_data_.serial_print_period_ms){
		String calibration_stage;
		switch (calibration_data_.calibration_state) {
			case CalibrationState::servoSelection:
			calibration_stage = "sel";
			break;
			case CalibrationState::zeroCalibration:
			calibration_stage = "zer";
			break;
			case CalibrationState::firstPointCalibration:
			calibration_stage = "min";
			break;
			case CalibrationState::secondPointCalibration:
			calibration_stage = "max";
			break;
		}

		Serial.println( "Calib. " + calibration_stage + " :"
		+ " | ang:" + (String)PCA9685_1_servo_map_[calibration_data_.selected_servo].get_assigned_anlge()
		+ " | zer:" + (String)PCA9685_1_servo_map_[calibration_data_.selected_servo].get_zero_offset()
		);

		calibration_data_.serial_print_last_millis = current_millis;
	}
}

void JointsManager::calibration_servo_selection(bool& sel_button_pressed, bool forward_button_pressed, bool back_button_pressed){

	if (forward_button_pressed){
		calibration_data_.selected_servo = calibration_data_.selected_servo + 1;
		if (calibration_data_.selected_servo > PCA9685_1_servo_map_.size() - 1) calibration_data_.selected_servo = 0;
	}
	else if (back_button_pressed){
		calibration_data_.selected_servo = calibration_data_.selected_servo - 1;
		if (calibration_data_.selected_servo < 0) calibration_data_.selected_servo = PCA9685_1_servo_map_.size() - 1;
	}
	if (sel_button_pressed){
		calibration_data_.calibration_state = CalibrationState::zeroCalibration;
	}
}

void JointsManager::calibration_zero_calibration(bool& sel_button_pressed, bool forward_button_pressed, bool back_button_pressed){

	if (forward_button_pressed){
		PCA9685_1_servo_map_[calibration_data_.selected_servo].clean_calibration_values();
	}
	if (back_button_pressed){
		calibration_data_.calibration_state = CalibrationState::servoSelection;
	}
	if (sel_button_pressed){
		calibration_data_.calibration_state = CalibrationState::firstPointCalibration;
		PCA9685_1_servo_map_[calibration_data_.selected_servo].calibration_set_zero(true,0);
	}
}

void JointsManager::calibration_first_point_calibration(bool& sel_button_pressed, bool forward_button_pressed, bool back_button_pressed){

	if (forward_button_pressed){
	}
	if (back_button_pressed){
		calibration_data_.calibration_state = CalibrationState::zeroCalibration;
	}
	if (sel_button_pressed){
		calibration_data_.calibration_state = CalibrationState::secondPointCalibration;
		PCA9685_1_servo_map_[calibration_data_.selected_servo].calibration_set_min_angle(true,0);
	}
}

void JointsManager::calibration_second_point_calibration(bool& sel_button_pressed, bool forward_button_pressed, bool back_button_pressed){

	if (forward_button_pressed){
	}
	if (back_button_pressed){
		calibration_data_.calibration_state = CalibrationState::firstPointCalibration;
	}
	if (sel_button_pressed){
		calibration_data_.calibration_state = CalibrationState::servoSelection;
		PCA9685_1_servo_map_[calibration_data_.selected_servo].calibration_set_max_angle(true,0);
		current_state_ = State::running;
	}
}
