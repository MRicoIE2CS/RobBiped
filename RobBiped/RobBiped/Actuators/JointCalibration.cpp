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

void JointsManager::calibration_setAngleToServo(uint16_t potentiometerVal){
	
	PCA9685_1_servoMap[calibrationData.selectedServo].setAngleTarget_rad(calibration_getAngleFromPotentiometer(potentiometerVal));
}

double JointsManager::calibration_getAngleFromPotentiometer(uint16_t potentiometerVal){
	
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

void JointsManager::calibrationModeEnterExitConditions(uint32_t currentMillis, bool squareButtonPressed, bool ThinButton1Pressed, bool ThinButton2Pressed){
	
	bool conditionsToEnterCalibration = (currentState == State::running) && (ThinButton1Pressed && ThinButton2Pressed);
	bool conditionsToExitCalibration = (currentState == State::calibrating) && (calibrationData.calibrationState == CalibrationState::servoSelection) && (ThinButton1Pressed && ThinButton2Pressed);
	if (conditionsToEnterCalibration && calibrationData.calibrationStateButtonChangeFlag == false) {
		currentState = State::calibrating;
		calibrationData.calibrationState = CalibrationState::servoSelection;
		calibrationData.calibrationStateButtonChangeFlag = true;
		calibrationData.lastMillisChangedCalibrationState = currentMillis;
	}
	else if (conditionsToExitCalibration && calibrationData.calibrationStateButtonChangeFlag == false){
		currentState = State::running;
		calibrationData.calibrationStateButtonChangeFlag = true;
		calibrationData.lastMillisChangedCalibrationState = currentMillis;
	}
}

void JointsManager::calibrationButtonPressedFlagMechanism(uint32_t currentMillis){
	
	if (calibrationData.calibrationStateButtonChangeFlag
	&& (abs(currentMillis - calibrationData.lastMillisChangedCalibrationState) > calibrationData.valueChangeDelay_ms)
	){
		calibrationData.calibrationStateButtonChangeFlag = false;
	}
}

void JointsManager::calibrationStateMachine(uint32_t currentMillis, bool squareButtonPressed, bool ThinButton1Pressed, bool ThinButton2Pressed){
	
	calibration_SerialPrint(currentMillis);
	
	if (calibrationData.calibrationStateButtonChangeFlag == false){
		
		if (squareButtonPressed || ThinButton1Pressed || ThinButton2Pressed){
			calibrationData.calibrationStateButtonChangeFlag = true;
			calibrationData.lastMillisChangedCalibrationState = currentMillis;
		}
		
		switch (calibrationData.calibrationState) {
			case CalibrationState::servoSelection:
			calibration_servoSelection(currentMillis, squareButtonPressed, ThinButton1Pressed, ThinButton2Pressed);
			break;
			case CalibrationState::zeroCalibration:
			calibration_zeroCalibration(currentMillis, squareButtonPressed, ThinButton1Pressed, ThinButton2Pressed);
			break;
			case CalibrationState::firstPointCalibration:
			calibration_firstPointCalibration(currentMillis, squareButtonPressed, ThinButton1Pressed, ThinButton2Pressed);
			break;
			case CalibrationState::secondPointCalibration:
			calibration_secondPointCalibration(currentMillis, squareButtonPressed, ThinButton1Pressed, ThinButton2Pressed);
			break;
		}
	}
}

void JointsManager::calibration_SerialPrint(uint32_t currentMillis){
	
	if (abs(currentMillis - calibrationData.SerialPrint_LastMillis) > calibrationData.SerialPrint_Period_ms){
		String calibrationStage;
		switch (calibrationData.calibrationState) {
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
		+ " | ang:" + (String)PCA9685_1_servoMap[calibrationData.selectedServo].getAssignedAnlge()
		+ " | zer:" + (String)PCA9685_1_servoMap[calibrationData.selectedServo].getZeroOffset()
		);
		
		calibrationData.SerialPrint_LastMillis = currentMillis;
	}
}

void JointsManager::calibration_servoSelection(uint32_t currentMillis, bool squareButtonPressed, bool ThinButton1Pressed, bool ThinButton2Pressed){
	
	if (ThinButton1Pressed){
		calibrationData.selectedServo = calibrationData.selectedServo + 1;
		if (calibrationData.selectedServo > PCA9685_1_servoMap.size() - 1) calibrationData.selectedServo = 0;
	}
	else if (ThinButton2Pressed){
		calibrationData.selectedServo = calibrationData.selectedServo - 1;
		if (calibrationData.selectedServo < 0) calibrationData.selectedServo = PCA9685_1_servoMap.size() - 1;
	}
	if (squareButtonPressed){
		calibrationData.calibrationState = CalibrationState::zeroCalibration;
	}
}

void JointsManager::calibration_zeroCalibration(uint32_t currentMillis, bool squareButtonPressed, bool ThinButton1Pressed, bool ThinButton2Pressed){
	
	if (ThinButton1Pressed){
		PCA9685_1_servoMap[calibrationData.selectedServo].cleanCalibrationValues();
	}
	if (ThinButton2Pressed){
		calibrationData.calibrationState = CalibrationState::servoSelection;
	}
	if (squareButtonPressed){
		calibrationData.calibrationState = CalibrationState::firstPointCalibration;
		PCA9685_1_servoMap[calibrationData.selectedServo].calibration_setZero(true,0);
	}
}

void JointsManager::calibration_firstPointCalibration(uint32_t currentMillis, bool squareButtonPressed, bool ThinButton1Pressed, bool ThinButton2Pressed){
	
	if (ThinButton1Pressed){
		
	}
	if (ThinButton2Pressed){
		calibrationData.calibrationState = CalibrationState::zeroCalibration;
	}
	if (squareButtonPressed){
		calibrationData.calibrationState = CalibrationState::secondPointCalibration;
		PCA9685_1_servoMap[calibrationData.selectedServo].calibration_setMinAngle(true,0);
	}
}

void JointsManager::calibration_secondPointCalibration(uint32_t currentMillis, bool squareButtonPressed, bool ThinButton1Pressed, bool ThinButton2Pressed){
	
	if (ThinButton1Pressed){
		
	}
	if (ThinButton2Pressed){
		calibrationData.calibrationState = CalibrationState::firstPointCalibration;
	}
	if (squareButtonPressed){
		calibrationData.calibrationState = CalibrationState::servoSelection;
		PCA9685_1_servoMap[calibrationData.selectedServo].calibration_setMaxAngle(true,0);
		currentState = State::running;
	}
}