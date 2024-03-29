/*
 * JointsManager.cpp
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

void JointsManager::init(){

	// Get Command singleton instance
	command_ = Command::get_instance();

	PCA9685_1_.begin();
	PCA9685_1_.setPWMFreq(50);  // Analog servos run at ~50 Hz updates
	sleep();

	joints_config();
}
	
void JointsManager::update(UserInput& _user_input){

	check_state(command_->commands.servo_onoff_toggle,
				command_->commands.servo_calibration_onoff_toggle,
				command_->commands.servo_selection_button_emulation,
				_user_input.get_digital_value(UserInput::DigitalInputList::forward_button),
				_user_input.get_digital_value(UserInput::DigitalInputList::back_button));
				
	if (current_state_ == State::calibrating){
		calibration_set_angle_to_servo(_user_input.get_analog_value(UserInput::AnalogInputList::potentiometer1));
	}

	servo_update();
}

// TODO: Document the flowchart of the modes and calibration modes/mechanism
void JointsManager::check_state(bool &_power_onoff_command, bool &_calibration_onoff, bool &_sel_button_pressed, bool forward_button_pressed, bool back_button_pressed){
	uint32_t current_millis = millis();

	// Run/sleep to servos
	if (change_state_conditions(current_millis, _power_onoff_command)) change_state(current_millis);

	// Calibration mode on/off
	calibration_mode_enter_exit_conditions(current_millis, _calibration_onoff);
	// Calibration state machine
	if (current_state_ == State::calibrating) calibration_state_machine(current_millis, _sel_button_pressed, forward_button_pressed, back_button_pressed);
	// Mechanism fro limiting the speed of mode change
	calibration_button_pressed_flag_mechanism(current_millis);

	_sel_button_pressed = false;
}

bool JointsManager::change_state_conditions(uint32_t& current_millis, bool& switch_command){

	bool conditions = switch_command && (current_state_ != State::calibrating);
	if (conditions) switch_command = false;
	return conditions;
}

void JointsManager::change_state(uint32_t& current_millis){
	last_millis_changed_state_ = current_millis;

	if (current_state_ == State::running){
		delay(500);
		sleep();
	}
	else if (current_state_ == State::sleeping){
		wakeup();
	}
}

void JointsManager::sleep(){

	PCA9685_1_.sleep();
	current_state_ = State::sleeping;
}

void JointsManager::wakeup(){

	PCA9685_1_.wakeup();
	current_state_ = State::running;
}

void JointsManager::servo_update(){
	
	if (current_state_ == State::sleeping) return;

	std::map<uint8_t, Joint>::iterator itMap;

	for (itMap = PCA9685_1_servo_map_.begin(); itMap!=PCA9685_1_servo_map_.end(); ++itMap){
		if (itMap->second.is_update_needed()) {
			PCA9685_1_.setPWM(itMap->first, 0, itMap->second.get_PWM_pulse_width_update());
			last_joint_setpoints_[static_cast<Configuration::JointsNames>(itMap->first)] = itMap->second.get_assigned_anlge();
		}
	}
}

bool JointsManager::set_angle_to_joint(Configuration::JointsNames _joint_index, double _servo_angle_rad)
{
	bool ret_val = PCA9685_1_servo_map_[static_cast<uint8_t>(_joint_index)].set_angle_target_rad(_servo_angle_rad);
	if (!ret_val)
	{
		if (command_->commands.show_alarm_angle_limit) Serial.println("Angle error joint " + (String)static_cast<uint8_t>(_joint_index) + ", angle:\t" + (String)_servo_angle_rad);
		return false;
	}
	return true;
}

bool JointsManager::revert_angle_to_joint(Configuration::JointsNames _joint_index)
{
	bool ret_val = PCA9685_1_servo_map_[static_cast<uint8_t>(_joint_index)].set_angle_target_rad(last_joint_setpoints_[_joint_index]);
	if (!ret_val) return false;
	return true;
}

JointsManager::State JointsManager::get_current_state()
{
	return current_state_;
}

std::map<Configuration::JointsNames, double> JointsManager::get_last_joint_setpoints()
{
	return last_joint_setpoints_;
}

double JointsManager::get_last_joint_setpoints(Configuration::JointsNames _joint)
{
	return last_joint_setpoints_[_joint];
}
