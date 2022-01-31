// UpdateServos.h

#ifndef _UPDATESERVOS_h
#define _UPDATESERVOS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include "../Executer/I_Task.h"
//#include "SignalGenerator.h"
#include "MG996R.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <map>
using namespace std;

class UpdateServos : public I_Task{
	
	private:
	
		Adafruit_PWMServoDriver PCA9685_1 = Adafruit_PWMServoDriver(0x40);
		
		
		std::map<unsigned short,MG996R> PCA9685_1_servoMap;
		
		int angleToPulse(int _ang);	// debug
	
	public:
	
	void init();
	
	void setAngleToServo(unsigned short servoIndex, double servoAngle);
	void setAngleToServo(unsigned short servoIndex, int servoAngle);
	
	void update();
	
};

#endif

