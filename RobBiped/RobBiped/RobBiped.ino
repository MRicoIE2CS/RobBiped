
/*
 * RobBiped.ino
 *
 * Created: 1/28/2022 8:02:12 PM
 * Author: MRICO
 */ 

#include "Main/Executer.h"
//#include <Wire.h>
//#include <Adafruit_PWMServoDriver.h>
//#include <vector>

Executer executer;

void setup()
{
	Serial.begin(500000);
	
	executer.init();
	//while(!(Serial.available())){};
	
	Serial.println("Initial delay...");
	delay(3000);
	Serial.println("3");
	delay(1000);
	Serial.println("2");
	delay(1000);
	Serial.println("1");
	delay(1000);
}

uint32_t currentMicros;
uint32_t elapsedMicros;

void loop()
{
	currentMicros = micros();
	
	executer.execution();
	
	elapsedMicros = micros() - currentMicros;
	//Serial.println("cycleTime: " + (String)(elapsedMicros));
}
