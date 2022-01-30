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
	servoInitializer.setAngleTarget(0);
	PCA9685_1_servoMap[1] = servoInitializer;
	
}

int targetAngle_ = 0;
bool direction = false;

void UpdateServos::update(){
	
	if (!direction)	{
		targetAngle_--;
		if (targetAngle_ < 0) direction = true;
	}
	else {
		targetAngle_++;
		if (targetAngle_ > 180) direction = false;
	}
	
	
	
// 	std::map<unsigned short,MG996R> PCA9685_1_servoMap;
// 	
// 	PCA9685_1_servoMap[0].setAngleTarget(targetAngle_);
	
	
	Serial.println("targetAngle_: " + (String)targetAngle_);
	int pulse = angleToPulse(targetAngle_);
	Serial.println("pulse: " + (String)pulse);
	PCA9685_1.setPWM(0, 0, pulse );
	
	
	//PCA9685_1.setPWM(1, 0, PCA9685_1_servoMap[1].getPulseWidth() );
}





int UpdateServos::angleToPulse(int _ang){
	int pulse = 100 + (_ang * 410 / 180);
	return pulse;
}