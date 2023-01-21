/*
 * JointsManager.h
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

#ifndef _UPDATESERVOS_h
#define _UPDATESERVOS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <map>

#include "../Main/I_PeriodicTask.h"
#include "Joint.h"
#include "MG996R.h"
#include "../UserInput/UserInput.h"
#include "../UserInput/Command.h"

class JointsManager : public I_PeriodicTask{
	
	private:
	
		// Serial Commands pointer
		Command* command_;
	
		Adafruit_PWMServoDriver PCA9685_1_ = Adafruit_PWMServoDriver(0x40);
		
		std::map<uint8_t,Joint> PCA9685_1_servo_map_;
		
		enum class State { running , calibrating, sleeping } current_state_;
		uint64_t last_millis_changed_state_;
		enum class CalibrationState { servoSelection, zeroCalibration, firstPointCalibration, secondPointCalibration };
		struct Calibration {
			CalibrationState calibration_state;
			int8_t selected_servo = 0;
			uint32_t last_millis_changed_calibration_state;
			uint32_t value_change_delay_ms = 2000;
			bool calibration_state_button_change_flag;
			uint32_t serial_print_last_millis;
			uint32_t serial_print_period_ms = 400;
		} calibration_data_;
		
		void check_state(bool& sel_button_pressed, bool forward_button_pressed, bool back_button_pressed);
		void calibration_mode_enter_exit_conditions(uint32_t current_millis, bool& sel_button_pressed, bool forward_button_pressed, bool back_button_pressed);
		void calibration_state_machine(uint32_t current_millis, bool& sel_button_pressed, bool forward_button_pressed, bool back_button_pressed);
		void calibration_button_pressed_flag_mechanism(uint32_t current_millis);
		void calibration_servo_selection(bool& sel_button_pressed, bool forward_button_pressed, bool back_button_pressed);
		void calibration_zero_calibration(bool& sel_button_pressed, bool forward_button_pressed, bool back_button_pressed);
		void calibration_first_point_calibration(bool& sel_button_pressed, bool forward_button_pressed, bool back_button_pressed);
		void calibration_second_point_calibration(bool& sel_button_pressed, bool forward_button_pressed, bool back_button_pressed);
		void calibration_serial_print(uint32_t current_millis);
		void calibration_set_angle_to_servo(uint16_t potentiometer_val);
		double calibration_get_angle_from_potentiometer(uint16_t potentiometer_val);

		void sleep();
		void wakeup();
		bool change_state_conditions(uint32_t& current_millis, bool& switch_command);
		void change_state(uint32_t& current_millis);

	public:

		void init();
		void joints_config();
		
		void set_angle_to_servo(uint8_t servo_index, double servo_angle);
		
		void update(UserInput& _user_input);
		void servo_update();
	
};

#endif

