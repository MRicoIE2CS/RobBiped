// UpdateServos.h

#ifndef _UPDATESERVOS_h
#define _UPDATESERVOS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include "../Executer/I_Task.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

class UpdateServos : public I_Task{
	
	private:
	
		Adafruit_PWMServoDriver board1 = Adafruit_PWMServoDriver(0x40);
		
		#define SERVOMIN  100 // this is the 'minimum' pulse length count (out of 4096)
		#define SERVOMAX  510 // this is the 'maximum' pulse length count (out of 4096)
		// 	#define SERVOMIN  125 // this is the 'minimum' pulse length count (out of 4096)
		// 	#define SERVOMAX  575 // this is the 'maximum' pulse length count (out of 4096)

		int servoNumber = 0;
		
		int angleToPulse(int ang);
		
		struct ServoValueAssigned
		{
			uint8_t index;
			int angle;
		};
		
		ServoValueAssigned servo0;
		
		struct ServoAngleTable
		{
			ServoValueAssigned servo0;
		}servoAngleTable;
	
	public:
	
	void init();
	
	bool setNextAngleValues(uint8_t servoNumber, int _ang);
	
	void update();
	
};

#endif

