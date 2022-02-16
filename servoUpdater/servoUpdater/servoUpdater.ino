
/*
 * servoUpdater.ino
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
	Serial.begin(9600);
	
	executer.init();

}

void loop()
{
	
	executer.execution();

}
