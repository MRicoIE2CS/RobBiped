// UpdateServos.h

#ifndef _UPDATESERVOS_h
#define _UPDATESERVOS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include "../Executer/I_Task.h"
#include "trialTrajectoryGenerator.h"
#include "MG996R.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <map>
using namespace std;

class UpdateServos : public I_Task{
	
	private:
	
		Adafruit_PWMServoDriver PCA9685_1 = Adafruit_PWMServoDriver(0x40);
		
		#define SERVOMIN  100 // this is the 'minimum' pulse length count (out of 4096)
		#define SERVOMAX  510 // this is the 'maximum' pulse length count (out of 4096)
		
		
		std::map<unsigned short,MG996R> PCA9685_1_servoMap;
	
	public:
	
	void init();
	
	bool setNextAngleValues(uint8_t servoNumber, double _ang);
	
	void update();
	
};

#endif

