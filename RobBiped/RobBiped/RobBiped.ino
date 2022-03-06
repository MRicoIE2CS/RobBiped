

/*
 * RobBiped.ino
 *
 * Created: 1/28/2022 8:02:12 PM
 * Author: MRICO
 */ 

#include "Main/Executer.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Executer executer;

void setup()
{
	Serial.begin(500000);
	
	executer.init();

}

void loop()
{
	//uint32_t initMicros = micros();
	
	executer.execution();
	
	//uint32_t finalMicros = micros();
	//Serial.println("cycleTime: " + (String)(finalMicros - initMicros));
}
