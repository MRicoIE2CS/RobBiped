// 
// 
// 

#include "UpdateServos.h"


void Servos::init(){
	
	

	PCA9685_1.begin();
	PCA9685_1.setPWMFreq(50);  // Analog servos run at ~50 Hz updates
	PCA9685_1.sleep();
	
	Joint jointInitializer;
	jointInitializer.setAngleTarget_rad(0.0);
	PCA9685_1_servoMap[0] = jointInitializer;
// 	jointInitializer.setAngleTarget(0);
// 	PCA9685_1_servoMap[1] = jointInitializer;
	
}

void Servos::assocButtons(uint8_t _pinbutton1, uint8_t _pinbutton2){
	
	pinbutton1 = _pinbutton1;
	pinbutton2 = _pinbutton2;
}

void Servos::sleep(){
	
	PCA9685_1.sleep();
	currentState = state::sleeping;
}

void Servos::wakeup(){
	
	PCA9685_1.wakeup();
	currentState = state::running;
}

void Servos::changeState(){
	
	unsigned long currentMillis = millis();
	if (currentMillis - lastMillisChangedState > 2000){
		if (currentState == state::running){
			this->sleep();
		}
		else if (currentState == state::sleeping){
			this->wakeup();
		}
		lastMillisChangedState = currentMillis;
	}
}
	
void Servos::update(){
	if (currentState == state::sleeping) return;
	
	std::map<unsigned short,Joint>::iterator itMap;
	
	for (itMap = PCA9685_1_servoMap.begin(); itMap!=PCA9685_1_servoMap.end(); ++itMap){
		if (itMap->second.isUpdateNeeded()) {
			PCA9685_1.setPWM(itMap->first, 0, itMap->second.getPWMPulseWidthUpdate() );
			//Serial.println("PulseWidthApplied: " + (String)itMap->second.getPulseWidthApplied());
		}
	}
}

void Servos::setAngleToServo(unsigned short servoIndex, double servoAngle){
	
	PCA9685_1_servoMap[servoIndex].setAngleTarget_rad(servoAngle);
}

// void Servos::setAngleToServo(unsigned short servoIndex, int servoAngle){
// 	
// 	PCA9685_1_servoMap[servoIndex].setAngleTarget_deg(servoAngle);
// }



// int UpdateServos::angleToPulse(int _ang){
// 	int pulse = 100 + (_ang * 410 / 180);
// 	return pulse;
// }