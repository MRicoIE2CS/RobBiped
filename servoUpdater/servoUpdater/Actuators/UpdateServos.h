// UpdateServos.h

#ifndef _UPDATESERVOS_h
#define _UPDATESERVOS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include "../Main/I_PeriodicTask.h"
//#include "SignalGenerator.h"
#include "MG996R.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <map>
using namespace std;

class UpdateServos : public I_PeriodicTask{
	
	private:
	
		Adafruit_PWMServoDriver PCA9685_1 = Adafruit_PWMServoDriver(0x40);
		
		std::map<unsigned short,MG996R> PCA9685_1_servoMap;
		
		enum class state { running , sleep } currentState;
			
		unsigned long lastMillisChangedState;
	
	public:
	
	void init();
	void sleep();
	void wakeup();
	void changeState();
	
	void setAngleToServo(unsigned short servoIndex, double servoAngle);
	void setAngleToServo(unsigned short servoIndex, int servoAngle);
	
	void update();
	
};

#endif

