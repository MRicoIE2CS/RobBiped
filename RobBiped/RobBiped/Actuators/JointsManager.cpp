// 
// 
// 

#include "JointsManager.h"


void JointsManager::init(){
	
	

	PCA9685_1.begin();
	PCA9685_1.setPWMFreq(50);  // Analog servos run at ~50 Hz updates
	sleep();
	
	jointsConfig();
}
	
void JointsManager::update(UserInput _userInput){
	
	checkState(_userInput.getDigitalValue(UserInput::DigitalInputList::squareButton),
				_userInput.getDigitalValue(UserInput::DigitalInputList::thinButton1),
				_userInput.getDigitalValue(UserInput::DigitalInputList::thinButton2));
				
	if (currentState == State::calibrating){
		calibration_setAngleToServo(_userInput.getAnalogValue(UserInput::AnalogInputList::potentiometer1));
	}
	
	servoUpdate();
}

void JointsManager::checkState(bool squareButtonPressed, bool ThinButton1Pressed, bool ThinButton2Pressed){
	uint32_t currentMillis = millis();
	
	if (changeStateConditions(currentMillis, squareButtonPressed)) changeState(currentMillis);	// run/sleep to servos
	
	calibrationModeEnterExitConditions(currentMillis, squareButtonPressed, ThinButton1Pressed, ThinButton2Pressed);
	if (currentState == State::calibrating) calibrationStateMachine(currentMillis, squareButtonPressed, ThinButton1Pressed, ThinButton2Pressed);
	calibrationButtonPressedFlagMechanism(currentMillis);
	
}

bool JointsManager::changeStateConditions(uint32_t currentMillis, bool squareButtonPressed){
	
	bool conditions = squareButtonPressed && (currentState != State::calibrating) 
						&& (calibrationData.calibrationStateButtonChangeFlag == false)
						&& (abs(currentMillis - lastMillisChangedState) > 2000);
	return conditions;
}

void JointsManager::changeState(uint32_t currentMillis){
	lastMillisChangedState = currentMillis;
	
	if (currentState == State::running){
		this->sleep();
	}
	else if (currentState == State::sleeping){
		this->wakeup();
	}
}

void JointsManager::sleep(){
	
	PCA9685_1.sleep();
	currentState = State::sleeping;
}

void JointsManager::wakeup(){
	
	PCA9685_1.wakeup();
	currentState = State::running;
}

void JointsManager::servoUpdate(){
	
	if (currentState == State::sleeping) return;
	
	std::map<unsigned short,Joint>::iterator itMap;
	
	for (itMap = PCA9685_1_servoMap.begin(); itMap!=PCA9685_1_servoMap.end(); ++itMap){
		if (itMap->second.isUpdateNeeded()) {
			PCA9685_1.setPWM(itMap->first, 0, itMap->second.getPWMPulseWidthUpdate() );
		}
	}
}

void JointsManager::setAngleToServo(unsigned short servoIndex, double servoAngle){
	
	if (currentState == State::running){
		Serial.println("Running: " + (String)servoAngle);
		PCA9685_1_servoMap[servoIndex].setAngleTarget_rad(servoAngle);
	}
	
}