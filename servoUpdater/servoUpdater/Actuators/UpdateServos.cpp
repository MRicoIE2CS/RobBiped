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
	servoInitializer.setAngleTarget(90);
	PCA9685_1_servoMap[1] = servoInitializer;
	
}

void UpdateServos::update(){
	
	PCA9685_1.setPWM(0, 0, PCA9685_1_servoMap[0].getPulseWidth() );
	PCA9685_1.setPWM(1, 0, PCA9685_1_servoMap[1].getPulseWidth() );
}

int UpdateServos::angleToPulse(double ang){
	int pulse = 300;//map((int)ang,0, 180, SERVOMIN,SERVOMAX);
	return pulse;
}