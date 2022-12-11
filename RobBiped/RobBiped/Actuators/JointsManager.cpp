// 
// 
// 

#include "JointsManager.h"


void JointsManager::init(){
	
	// Get Command singleton instance
	command = Command::getInstance();

	PCA9685_1.begin();
	PCA9685_1.setPWMFreq(50);  // Analog servos run at ~50 Hz updates
	sleep();
	
	jointsConfig();
}
	
void JointsManager::update(UserInput _userInput){
	
	checkState(command->commands.servo_selection_button_emulation,
				_userInput.getDigitalValue(UserInput::DigitalInputList::thinButton1),
				_userInput.getDigitalValue(UserInput::DigitalInputList::thinButton2));
				
	if (currentState == State::calibrating){
		calibration_setAngleToServo(_userInput.getAnalogValue(UserInput::AnalogInputList::potentiometer1));
	}
	
	servoUpdate();
}

void JointsManager::checkState(bool& sel_button_pressed, bool ThinButton1Pressed, bool ThinButton2Pressed){
	uint32_t currentMillis = millis();
	
	if (changeStateConditions(currentMillis, command->commands.servo_onoff_toggle)) changeState(currentMillis);	// run/sleep to servos
	
	calibrationModeEnterExitConditions(currentMillis, sel_button_pressed, ThinButton1Pressed, ThinButton2Pressed);
	if (currentState == State::calibrating) calibrationStateMachine(currentMillis, sel_button_pressed, ThinButton1Pressed, ThinButton2Pressed);
	calibrationButtonPressedFlagMechanism(currentMillis);
	
	sel_button_pressed = false;
}

bool JointsManager::changeStateConditions(uint32_t& currentMillis, bool& switchCommand){
	
	bool conditions = switchCommand && (currentState != State::calibrating);
						//&& (calibrationData.calibrationStateButtonChangeFlag == false)
						//&& (abs(currentMillis - lastMillisChangedState) > 2000);
	if (conditions) switchCommand = false;
	return conditions;
}

void JointsManager::changeState(uint32_t& currentMillis){
	lastMillisChangedState = currentMillis;
	
	if (currentState == State::running){
		sleep();
	}
	else if (currentState == State::sleeping){
		wakeup();
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