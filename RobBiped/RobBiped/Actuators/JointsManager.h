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
		Command* command;
	
		Adafruit_PWMServoDriver PCA9685_1 = Adafruit_PWMServoDriver(0x40);
		
		std::map<unsigned short,Joint> PCA9685_1_servoMap;
		
		enum class State { running , calibrating, sleeping } currentState;
		unsigned long lastMillisChangedState;
		enum class CalibrationState { servoSelection, zeroCalibration, firstPointCalibration, secondPointCalibration };
		struct Calibration {
			CalibrationState calibrationState;
			int8_t selectedServo = 0;
			uint32_t lastMillisChangedCalibrationState;
			uint32_t valueChangeDelay_ms = 2000;
			bool calibrationStateButtonChangeFlag;
			uint32_t SerialPrint_LastMillis;
			uint32_t SerialPrint_Period_ms = 400;
		} calibrationData;
		
		void checkState(bool& sel_button_pressed, bool ThinButton1Pressed, bool ThinButton2Pressed);
		void calibrationModeEnterExitConditions(uint32_t currentMillis, bool squareButtonPressed, bool ThinButton1Pressed, bool ThinButton2Pressed);
		void calibrationStateMachine(uint32_t currentMillis, bool squareButtonPressed, bool ThinButton1Pressed, bool ThinButton2Pressed);
		void calibrationButtonPressedFlagMechanism(uint32_t currentMillis);
		void calibration_servoSelection(uint32_t currentMillis, bool squareButtonPressed, bool ThinButton1Pressed, bool ThinButton2Pressed);
		void calibration_zeroCalibration(uint32_t currentMillis, bool squareButtonPressed, bool ThinButton1Pressed, bool ThinButton2Pressed);
		void calibration_firstPointCalibration(uint32_t currentMillis, bool squareButtonPressed, bool ThinButton1Pressed, bool ThinButton2Pressed);
		void calibration_secondPointCalibration(uint32_t currentMillis, bool squareButtonPressed, bool ThinButton1Pressed, bool ThinButton2Pressed);
		void calibration_SerialPrint(uint32_t currentMillis);
		void sleep();
		void wakeup();
		bool changeStateConditions(uint32_t& currentMillis, bool& switchCommand);
		void changeState(uint32_t& currentMillis);
		
		void calibration_setAngleToServo(uint16_t potentiometerVal);
		double calibration_getAngleFromPotentiometer(uint16_t potentiometerVal);
	
	public:
	
		void init();
		void jointsConfig();
		void assocButtons(uint8_t _pinbutton1, uint8_t _pinbutton2);
		
		void setAngleToServo(unsigned short servoIndex, double servoAngle);
		void setAngleToServo(unsigned short servoIndex, int servoAngle);
		
		void update(UserInput _userInput);
		void servoUpdate();
	
};

#endif

