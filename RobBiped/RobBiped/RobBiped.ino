
/*
 * RobBiped.ino
 *
 * Created: 1/28/2022 8:02:12 PM
 * Author: MRICO
 */ 

#include "Main/Executer.h"

Executer executer;

void setup()
{
	Serial.begin(500000);
	
	executer.init();
	
	delay(1000);
	Serial.println("Initial delay...");
	delay(3000);
	
	Serial.println("3");
	delay(1000);
	Serial.println("2");
	delay(1000);
	Serial.println("1");
	delay(1000);
}

void loop()
{
	executer.execution();
}
