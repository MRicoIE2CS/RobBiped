// 
// 
// 

#include "UpdateServos.h"


void UpdateServos::init(){
	
	

	board1.begin();
	board1.setPWMFreq(60);  // Analog servos run at ~50 Hz updates
	
	
	servo0.index = 0;
	servo0.angle = 90;
	
}

bool UpdateServos::setNextAngleValues(uint8_t servoNumber, int _ang){
	if ((_ang < 0 || _ang > 180) || (servoNumber <0 || servoNumber > 15))
	{
		return true;
	}
	else {
		servo0.angle = _ang;
		return false;
	}
}

int currentAngle = 0;
bool direction = false;

void UpdateServos::update(){
	
	
	
// 	Serial.println("_________\n");
// 	Serial.println("currentAngle:" + (String)currentAngle);
	
	setNextAngleValues(0, currentAngle);
	board1.setPWM(servo0.index, 0, angleToPulse(servo0.angle) );
	
	if (direction) currentAngle = currentAngle + 1;
	else currentAngle = currentAngle - 1;
	if (currentAngle >= 180)
	{
		direction = false;
	}
	if (currentAngle <= 0)
	{
		direction = true;
	}
	//delay(1500);
}

int UpdateServos::angleToPulse(int ang){
	int pulse = map(ang,0, 180, SERVOMIN,SERVOMAX);// map angle of 0 to 180 to Servo min and Servo max
// 	Serial.print("Angle: ");Serial.print(ang);
// 	Serial.print(" pulse: ");Serial.println(pulse);
	return pulse;
}