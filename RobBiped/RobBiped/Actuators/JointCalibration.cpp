/*
 * JointsConfig.cpp
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

#include "JointsManager.h"

void JointsManager::calibration_set_angle_to_servo(uint16_t potentiometerVal){
	
	PCA9685_1_servoMap_[calibration_data_.selectedServo].set_angle_target_rad(calibration_get_angle_from_potentiometer(potentiometerVal));
}

double JointsManager::calibration_get_angle_from_potentiometer(uint16_t potentiometerVal){
	
	double readingAngle_0;
	if (potentiometerVal < 1000){
		readingAngle_0 = 0;
	}
	else if (potentiometerVal > 3000){
		readingAngle_0 = PI;
	}
	else {
		readingAngle_0 = (double)(potentiometerVal - 1000) / (double)2000 * PI;
	}
	double nextAngle_0 = readingAngle_0 - HALF_PI;
	
	return nextAngle_0;
}

void JointsManager::calibration_mode_enter_exit_conditions(uint32_t currentMillis, bool squareButtonPressed, bool ThinButton1Pressed, bool ThinButton2Pressed){
	
	bool conditionsToEnterCalibration = (current_state_ == State::running) && (ThinButton1Pressed && ThinButton2Pressed);
	bool conditionsToExitCalibration = (current_state_ == State::calibrating) && (calibration_data_.calibrationState == CalibrationState::servoSelection) && (ThinButton1Pressed && ThinButton2Pressed);
	if (conditionsToEnterCalibration && calibration_data_.calibrationStateButtonChangeFlag == false) {
		current_state_ = State::calibrating;
		calibration_data_.calibrationState = CalibrationState::servoSelection;
		calibration_data_.calibrationStateButtonChangeFlag = true;
		calibration_data_.lastMillisChangedCalibrationState = currentMillis;
	}
	else if (conditionsToExitCalibration && calibration_data_.calibrationStateButtonChangeFlag == false){
		current_state_ = State::running;
		calibration_data_.calibrationStateButtonChangeFlag = true;
		calibration_data_.lastMillisChangedCalibrationState = currentMillis;
	}
}

void JointsManager::calibration_button_pressed_flag_mechanism(uint32_t currentMillis){
	
	if (calibration_data_.calibrationStateButtonChangeFlag
	&& (abs(currentMillis - calibration_data_.lastMillisChangedCalibrationState) > calibration_data_.valueChangeDelay_ms)
	){
		calibration_data_.calibrationStateButtonChangeFlag = false;
	}
}

void JointsManager::calibration_state_machine(uint32_t currentMillis, bool squareButtonPressed, bool ThinButton1Pressed, bool ThinButton2Pressed){
	
	calibration_serial_print(currentMillis);
	
	if (calibration_data_.calibrationStateButtonChangeFlag == false){
		
		if (squareButtonPressed || ThinButton1Pressed || ThinButton2Pressed){
			calibration_data_.calibrationStateButtonChangeFlag = true;
			calibration_data_.lastMillisChangedCalibrationState = currentMillis;
		}
		
		switch (calibration_data_.calibrationState) {
			case CalibrationState::servoSelection:
			calibration_servo_selection(currentMillis, squareButtonPressed, ThinButton1Pressed, ThinButton2Pressed);
			break;
			case CalibrationState::zeroCalibration:
			calibration_zero_calibration(currentMillis, squareButtonPressed, ThinButton1Pressed, ThinButton2Pressed);
			break;
			case CalibrationState::firstPointCalibration:
			calibration_first_point_calibration(currentMillis, squareButtonPressed, ThinButton1Pressed, ThinButton2Pressed);
			break;
			case CalibrationState::secondPointCalibration:
			calibration_second_point_calibration(currentMillis, squareButtonPressed, ThinButton1Pressed, ThinButton2Pressed);
			break;
		}
	}
}

void JointsManager::calibration_serial_print(uint32_t currentMillis){
	
	if (abs(currentMillis - calibration_data_.SerialPrint_LastMillis) > calibration_data_.SerialPrint_Period_ms){
		String calibrationStage;
		switch (calibration_data_.calibrationState) {
			case CalibrationState::servoSelection:
			calibrationStage = "sel";
			break;
			case CalibrationState::zeroCalibration:
			calibrationStage = "zer";
			break;
			case CalibrationState::firstPointCalibration:
			calibrationStage = "min";
			break;
			case CalibrationState::secondPointCalibration:
			calibrationStage = "max";
			break;
		}
		
		Serial.println( "Calib. " + calibrationStage + " :"
		+ " | ang:" + (String)PCA9685_1_servoMap_[calibration_data_.selectedServo].get_assigned_anlge()
		+ " | zer:" + (String)PCA9685_1_servoMap_[calibration_data_.selectedServo].get_zero_offset()
		);
		
		calibration_data_.SerialPrint_LastMillis = currentMillis;
	}
}

void JointsManager::calibration_servo_selection(uint32_t currentMillis, bool squareButtonPressed, bool ThinButton1Pressed, bool ThinButton2Pressed){
	
	if (ThinButton1Pressed){
		calibration_data_.selectedServo = calibration_data_.selectedServo + 1;
		if (calibration_data_.selectedServo > PCA9685_1_servoMap_.size() - 1) calibration_data_.selectedServo = 0;
	}
	else if (ThinButton2Pressed){
		calibration_data_.selectedServo = calibration_data_.selectedServo - 1;
		if (calibration_data_.selectedServo < 0) calibration_data_.selectedServo = PCA9685_1_servoMap_.size() - 1;
	}
	if (squareButtonPressed){
		calibration_data_.calibrationState = CalibrationState::zeroCalibration;
	}
}

void JointsManager::calibration_zero_calibration(uint32_t currentMillis, bool squareButtonPressed, bool ThinButton1Pressed, bool ThinButton2Pressed){
	
	if (ThinButton1Pressed){
		PCA9685_1_servoMap_[calibration_data_.selectedServo].clean_calibration_values();
	}
	if (ThinButton2Pressed){
		calibration_data_.calibrationState = CalibrationState::servoSelection;
	}
	if (squareButtonPressed){
		calibration_data_.calibrationState = CalibrationState::firstPointCalibration;
		PCA9685_1_servoMap_[calibration_data_.selectedServo].calibration_set_zero(true,0);
	}
}

void JointsManager::calibration_first_point_calibration(uint32_t currentMillis, bool squareButtonPressed, bool ThinButton1Pressed, bool ThinButton2Pressed){
	
	if (ThinButton1Pressed){
		
	}
	if (ThinButton2Pressed){
		calibration_data_.calibrationState = CalibrationState::zeroCalibration;
	}
	if (squareButtonPressed){
		calibration_data_.calibrationState = CalibrationState::secondPointCalibration;
		PCA9685_1_servoMap_[calibration_data_.selectedServo].calibration_set_min_angle(true,0);
	}
}

void JointsManager::calibration_second_point_calibration(uint32_t currentMillis, bool squareButtonPressed, bool ThinButton1Pressed, bool ThinButton2Pressed){
	
	if (ThinButton1Pressed){
		
	}
	if (ThinButton2Pressed){
		calibration_data_.calibrationState = CalibrationState::firstPointCalibration;
	}
	if (squareButtonPressed){
		calibration_data_.calibrationState = CalibrationState::servoSelection;
		PCA9685_1_servoMap_[calibration_data_.selectedServo].calibration_set_max_angle(true,0);
		current_state_ = State::running;
	}
}