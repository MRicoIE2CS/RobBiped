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
	
		Adafruit_PWMServoDriver board1 = Adafruit_PWMServoDriver(0x40);
		
		#define SERVOMIN  100 // this is the 'minimum' pulse length count (out of 4096)
		#define SERVOMAX  510 // this is the 'maximum' pulse length count (out of 4096)
		
		
		

		unsigned short servoNumber = 0;
		
		int angleToPulse(double ang);
		
		struct ServoValueAssigned
		{
			unsigned short index;
			double angle;
		};
		
		//ServoValueAssigned servo0;
		
		struct ServoAngleTable
		{
			ServoValueAssigned servo0;
			ServoValueAssigned servo1;
		}servoAngleTable;
		
		std::map<int,int> servomap;
	
	public:
	
	void init();
	
	bool setNextAngleValues(uint8_t servoNumber, double _ang);
	
	void update();
	
};

#endif

