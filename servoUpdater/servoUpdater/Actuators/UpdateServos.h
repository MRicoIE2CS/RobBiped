// UpdateServos.h

#ifndef _UPDATESERVOS_h
#define _UPDATESERVOS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include "../Main/I_PeriodicTask.h"
#include "Joint.h"
#include "MG996R.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <map>
using namespace std;
#include "../UserInput/UserInput.h"

class Servos : public I_PeriodicTask{
	
	private:
	
		Adafruit_PWMServoDriver PCA9685_1 = Adafruit_PWMServoDriver(0x40);
		
		std::map<unsigned short,Joint> PCA9685_1_servoMap;
		
		enum class State { running , calibrating, sleeping } currentState;
		unsigned long lastMillisChangedState;
		enum class CalibrationState { servoSelection, zeroCalibration, firstPointCalibration, secondPointCalibration };
		struct Calibration {
			CalibrationState calibrationState;
			int8_t selectedServo = 0;
			uint32_t lastMillisChangedCalibrationState;
			uint32_t valueChangeDelay_ms = 1000;
			bool calibrationStateButtonChangeFlag;
		} calibrationData;
		
		void checkState(bool squareButtonPressed, bool ThinButton1Pressed, bool ThinButton2Pressed);
		void calibrationModeEnterExitConditions(uint32_t currentMillis, bool squareButtonPressed, bool ThinButton1Pressed, bool ThinButton2Pressed);
		void calibrationStateMachine(uint32_t currentMillis, bool squareButtonPressed, bool ThinButton1Pressed, bool ThinButton2Pressed);
		void calibrationButtonPressedFlagMechanism(uint32_t currentMillis);
		void calibration_servoSelection(uint32_t currentMillis, bool squareButtonPressed, bool ThinButton1Pressed, bool ThinButton2Pressed);
		void calibration_zeroCalibration(uint32_t currentMillis, bool squareButtonPressed, bool ThinButton1Pressed, bool ThinButton2Pressed);
		void calibration_firstPointCalibration(uint32_t currentMillis, bool squareButtonPressed, bool ThinButton1Pressed, bool ThinButton2Pressed);
		void calibration_secondPointCalibration(uint32_t currentMillis, bool squareButtonPressed, bool ThinButton1Pressed, bool ThinButton2Pressed);
		void sleep();
		void wakeup();
		void changeState(uint32_t currentMillis);
		
		void calibration_setAngleToServo(UserInput _userInput);
		double calibration_getAngleFromPotentiometer(UserInput _userInput);
	
	public:
	
		void init();
		void servosConfig();
		void assocButtons(uint8_t _pinbutton1, uint8_t _pinbutton2);
		
		void setAngleToServo(unsigned short servoIndex, double servoAngle);
		void setAngleToServo(unsigned short servoIndex, int servoAngle);
		
		void update(UserInput _userInput);
		void servoUpdate();
	
};

#endif

