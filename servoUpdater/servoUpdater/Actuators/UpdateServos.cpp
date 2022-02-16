// 
// 
// 

#include "UpdateServos.h"


void UpdateServos::init(){
	
	

	PCA9685_1.begin();
	PCA9685_1.setPWMFreq(50);  // Analog servos run at ~50 Hz updates
	
	MG996R servoInitializer;
	servoInitializer.setAngleTarget(90);
	PCA9685_1_servoMap[0] = servoInitializer;
// 	servoInitializer.setAngleTarget(0);
// 	PCA9685_1_servoMap[1] = servoInitializer;
	
}

void UpdateServos::update(){
	
	std::map<unsigned short,MG996R>::iterator itMap;
	
	for (itMap = PCA9685_1_servoMap.begin(); itMap!=PCA9685_1_servoMap.end(); ++itMap){
		if (itMap->second.isNewPulseWidth()) {
			PCA9685_1.setPWM(itMap->first, 0, itMap->second.getPulseWidthToSend() );
			//Serial.println("PulseWidthApplied: " + (String)itMap->second.getPulseWidthApplied());
		}
	}
}

void UpdateServos::setAngleToServo(unsigned short servoIndex, double servoAngle){
	
	PCA9685_1_servoMap[servoIndex].setAngleTarget(servoAngle);
}

void UpdateServos::setAngleToServo(unsigned short servoIndex, int servoAngle){
	
	PCA9685_1_servoMap[servoIndex].setAngleTarget(servoAngle);
}



int UpdateServos::angleToPulse(int _ang){
	int pulse = 100 + (_ang * 410 / 180);
	return pulse;
}