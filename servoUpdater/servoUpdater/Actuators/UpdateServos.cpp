// 
// 
// 

#include "UpdateServos.h"


void UpdateServos::init(){
	
	

	board1.begin();
	board1.setPWMFreq(50);  // Analog servos run at ~50 Hz updates
	
	
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

void UpdateServos::update(){
	
	lastTimeExecutedMillis = millis();
	
	Serial.println("_________\n");
	Serial.println("currentAngle:" + (String)currentAngle);
	
	setNextAngleValues(0, currentAngle);
	board1.setPWM(servo0.index, 0, angleToPulse(servo0.angle) );
	
	currentAngle++;
	if (currentAngle > 180)
	{
		currentAngle = 0;
	}
	//delay(1500);
}

int UpdateServos::angleToPulse(int ang){
	int pulse = map(ang,0, 180, SERVOMIN,SERVOMAX);// map angle of 0 to 180 to Servo min and Servo max
	Serial.print("Angle: ");Serial.print(ang);
	Serial.print(" pulse: ");Serial.println(pulse);
	return pulse;
}