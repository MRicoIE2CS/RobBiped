// 
// 
// 

#include "UpdateServos.h"


void UpdateServos::init(){
	
	

	board1.begin();
	board1.setPWMFreq(50);  // Analog servos run at ~50 Hz updates
	
	
	servoAngleTable.servo0.index = 0;
	servoAngleTable.servo0.angle = 90;
	
}

void UpdateServos::update(){
	
	board1.setPWM(servoAngleTable.servo0.index, 0, angleToPulse(servoAngleTable.servo0.angle) );
}

int UpdateServos::angleToPulse(double ang){
	int pulse = 300;//map((int)ang,0, 180, SERVOMIN,SERVOMAX);
	return pulse;
}