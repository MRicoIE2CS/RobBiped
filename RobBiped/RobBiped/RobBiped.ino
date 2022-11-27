
/*
 * RobBiped.ino
 *
 * Created: 1/28/2022 8:02:12 PM
 * Author: MRICO
 */ 

#include "Main/Executer.h"
#include "UserInput/SerialCommand.h"

Executer executer;

SerialCommand* serialCommand = SerialCommand::getInstance();

void setup()
{
	Serial.begin(500000);
	
	executer.init();
	
	while (!serialCommand->commands.init)
	{
		Serial.println("type in 'init' to initialize");
		serialCommand->listenForCommands();
		delay(1000);
	}
	Serial.println("Initialize execution!");
}

void loop()
{
	executer.execution();
	
	serialCommand->listenForCommands();
}
