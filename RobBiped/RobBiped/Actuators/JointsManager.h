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
		
		std::map<unsigned short,Joint> PCA9685_1_servoMap_;
		
		enum class State { running , calibrating, sleeping } current_state_;
		unsigned long last_millis_changed_state_;
		enum class CalibrationState { servoSelection, zeroCalibration, firstPointCalibration, secondPointCalibration };
		struct Calibration {
			CalibrationState calibrationState;
			int8_t selectedServo = 0;
			uint32_t lastMillisChangedCalibrationState;
			uint32_t valueChangeDelay_ms = 2000;
			bool calibrationStateButtonChangeFlag;
			uint32_t SerialPrint_LastMillis;
			uint32_t SerialPrint_Period_ms = 400;
		} calibration_data_;
		
		void check_state(bool& sel_button_pressed, bool ThinButton1Pressed, bool ThinButton2Pressed);
		void calibration_mode_enter_exit_conditions(uint32_t currentMillis, bool squareButtonPressed, bool ThinButton1Pressed, bool ThinButton2Pressed);
		void calibration_state_machine(uint32_t currentMillis, bool squareButtonPressed, bool ThinButton1Pressed, bool ThinButton2Pressed);
		void calibration_button_pressed_flag_mechanism(uint32_t currentMillis);
		void calibration_servo_selection(uint32_t currentMillis, bool squareButtonPressed, bool ThinButton1Pressed, bool ThinButton2Pressed);
		void calibration_zero_calibration(uint32_t currentMillis, bool squareButtonPressed, bool ThinButton1Pressed, bool ThinButton2Pressed);
		void calibration_first_point_calibration(uint32_t currentMillis, bool squareButtonPressed, bool ThinButton1Pressed, bool ThinButton2Pressed);
		void calibration_second_point_calibration(uint32_t currentMillis, bool squareButtonPressed, bool ThinButton1Pressed, bool ThinButton2Pressed);
		void calibration_serial_print(uint32_t currentMillis);
		void sleep();
		void wakeup();
		bool change_state_conditions(uint32_t& currentMillis, bool& switchCommand);
		void change_state(uint32_t& currentMillis);
		
		void calibration_set_angle_to_servo(uint16_t potentiometerVal);
		double calibration_get_angle_from_potentiometer(uint16_t potentiometerVal);
	
	public:
	
		void init();
		void joints_config();
		
		void set_angle_to_servo(unsigned short servoIndex, double servoAngle);
		
		void update(UserInput& _userInput);
		void servo_update();
	
};

#endif

