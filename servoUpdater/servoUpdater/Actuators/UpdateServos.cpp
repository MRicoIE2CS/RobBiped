// 
// 
// 

#include "UpdateServos.h"


void Servos::init(){
	
	

	PCA9685_1.begin();
	PCA9685_1.setPWMFreq(50);  // Analog servos run at ~50 Hz updates
	PCA9685_1.sleep();
	
	servosConfig();
}

void Servos::servosConfig(){
	
	Joint jointInitializer;
	jointInitializer.setAngleTarget_rad(0.0);
	PCA9685_1_servoMap[0] = jointInitializer;
	// 	jointInitializer.setAngleTarget(0);
	// 	PCA9685_1_servoMap[1] = jointInitializer;
}
	
void Servos::update(UserInput _userInput){
	
	checkState(_userInput.getDigitalValue(UserInput::DigitalInputList::squareButton),
				_userInput.getDigitalValue(UserInput::DigitalInputList::thinButton1),
				_userInput.getDigitalValue(UserInput::DigitalInputList::thinButton2));
				
	if (currentState == State::calibrating){
		calibration_setAngleToServo(_userInput);
	}
	
	servoUpdate();
}

void Servos::checkState(bool squareButtonPressed, bool ThinButton1Pressed, bool ThinButton2Pressed){
	uint32_t currentMillis = millis();
	
	if (squareButtonPressed && currentState != State::calibrating) changeState(currentMillis);	// run/sleep to servos
	
	calibrationModeEnterExitConditions(currentMillis, squareButtonPressed, ThinButton1Pressed, ThinButton2Pressed);
	if (currentState == State::calibrating) calibrationStateMachine(currentMillis, squareButtonPressed, ThinButton1Pressed, ThinButton2Pressed);
	calibrationButtonPressedFlagMechanism(currentMillis);
	
}

void Servos::changeState(uint32_t currentMillis){
	
	if (abs(currentMillis - lastMillisChangedState) > 2000){
		lastMillisChangedState = currentMillis;
		
		if (currentState == State::running){
			this->sleep();
		}
		else if (currentState == State::sleeping){
			this->wakeup();
		}
	}
}

void Servos::sleep(){
	
	PCA9685_1.sleep();
	currentState = State::sleeping;
}

void Servos::wakeup(){
	
	PCA9685_1.wakeup();
	currentState = State::running;
}

void Servos::calibrationModeEnterExitConditions(uint32_t currentMillis, bool squareButtonPressed, bool ThinButton1Pressed, bool ThinButton2Pressed){
	
	bool conditionsToEnterCalibration = (currentState == State::running) && (ThinButton1Pressed && ThinButton2Pressed);
	if (conditionsToEnterCalibration && calibrationData.calibrationStateButtonChangeFlag == false) {
		currentState = State::calibrating;
		calibrationData.calibrationState = CalibrationState::servoSelection;
		calibrationData.calibrationStateButtonChangeFlag = true;
		calibrationData.lastMillisChangedCalibrationState = currentMillis;
	}
	bool conditionsToExitCalibration = (currentState == State::calibrating) &&(calibrationData.calibrationState == CalibrationState::servoSelection) && (ThinButton1Pressed && ThinButton2Pressed);
	if (conditionsToExitCalibration && calibrationData.calibrationStateButtonChangeFlag == false){
		currentState = State::running;
		calibrationData.calibrationStateButtonChangeFlag = true;
		calibrationData.lastMillisChangedCalibrationState = currentMillis;
	}
}

void Servos::calibrationButtonPressedFlagMechanism(uint32_t currentMillis){
	
	if (calibrationData.calibrationStateButtonChangeFlag && abs(calibrationData.lastMillisChangedCalibrationState - currentMillis) > calibrationData.valueChangeDelay_ms){
		calibrationData.calibrationStateButtonChangeFlag = false;
	}
}

void Servos::calibrationStateMachine(uint32_t currentMillis, bool squareButtonPressed, bool ThinButton1Pressed, bool ThinButton2Pressed){
	
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

void Servos::calibration_servoSelection(uint32_t currentMillis, bool squareButtonPressed, bool ThinButton1Pressed, bool ThinButton2Pressed){
	
	if (ThinButton1Pressed){
		calibrationData.selectedServo = calibrationData.selectedServo + 1;
		if (calibrationData.selectedServo > PCA9685_1_servoMap.size() - 1) calibrationData.selectedServo = 0;
	}
	else if (ThinButton2Pressed){
		calibrationData.selectedServo = calibrationData.selectedServo - 1;
		if (calibrationData.selectedServo < 0) calibrationData.selectedServo = PCA9685_1_servoMap.size() - 1;
	}
	
	if (squareButtonPressed){
		//TODO
	}
}

void Servos::calibration_zeroCalibration(uint32_t currentMillis, bool squareButtonPressed, bool ThinButton1Pressed, bool ThinButton2Pressed){
	
	//TODO
}

void Servos::calibration_firstPointCalibration(uint32_t currentMillis, bool squareButtonPressed, bool ThinButton1Pressed, bool ThinButton2Pressed){
	
	//TODO
}

void Servos::calibration_secondPointCalibration(uint32_t currentMillis, bool squareButtonPressed, bool ThinButton1Pressed, bool ThinButton2Pressed){
	
	//TODO
}

void Servos::servoUpdate(){
	
	if (currentState == State::sleeping) return;
	
	std::map<unsigned short,Joint>::iterator itMap;
	
	for (itMap = PCA9685_1_servoMap.begin(); itMap!=PCA9685_1_servoMap.end(); ++itMap){
		if (itMap->second.isUpdateNeeded()) {
			PCA9685_1.setPWM(itMap->first, 0, itMap->second.getPWMPulseWidthUpdate() );
			Serial.println("PulseWidthApplied: " + (String)itMap->second.getPWMPulseWidthUpdate());
		}
	}
}

void Servos::setAngleToServo(unsigned short servoIndex, double servoAngle){
	
	if (currentState == State::running){
		PCA9685_1_servoMap[servoIndex].setAngleTarget_rad(servoAngle);
	}
	
}

void Servos::calibration_setAngleToServo(UserInput _userInput){
	
	PCA9685_1_servoMap[calibrationData.selectedServo].setAngleTarget_rad(calibration_getAngleFromPotentiometer(_userInput));
}

double Servos::calibration_getAngleFromPotentiometer(UserInput _userInput){
	
	uint16_t pot1Val = _userInput.getAnalogValue(UserInput::AnalogInputList::potentiometer1);
	
	double readingAngle_0;
	if (pot1Val < 1000){
		readingAngle_0 = 0;
	}
	else if (pot1Val > 3000){
		readingAngle_0 = PI;
	}
	else {
		readingAngle_0 = (double)(pot1Val - 1000) / (double)2000 * PI;
	}
	double nextAngle_0 = readingAngle_0 - HALF_PI;
	
	return nextAngle_0;
}

// void Servos::setAngleToServo(unsigned short servoIndex, int servoAngle){
// 	
// 	PCA9685_1_servoMap[servoIndex].setAngleTarget_deg(servoAngle);
// }



// int UpdateServos::angleToPulse(int _ang){
// 	int pulse = 100 + (_ang * 410 / 180);
// 	return pulse;
// }