// UpdateServos.h

#ifndef _UPDATESERVOS_h
#define _UPDATESERVOS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include "../Main/I_PeriodicTask.h"
#include "Joint.h"
#include "MG996R.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <map>
using namespace std;

class Servos : public I_PeriodicTask{
	
	private:
	
		Adafruit_PWMServoDriver PCA9685_1 = Adafruit_PWMServoDriver(0x40);
		
		std::map<unsigned short,Joint> PCA9685_1_servoMap;
		
		enum class state { running , calibrating, sleeping } currentState;
			
		unsigned long lastMillisChangedState;
		
		uint8_t pinbutton1;
		uint8_t pinbutton2;
	
	public:
	
	void init();
	void servosConfig();
	void assocButtons(uint8_t _pinbutton1, uint8_t _pinbutton2);
	
	void sleep();
	void wakeup();
	void changeState();
	
	void setAngleToServo(unsigned short servoIndex, double servoAngle);
	void setAngleToServo(unsigned short servoIndex, int servoAngle);
	
	void update();
	
};

#endif

