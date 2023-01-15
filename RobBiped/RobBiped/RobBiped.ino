
/*
 * RobBiped.ino
 *
 * Created: 1/28/2022 8:02:12 PM
 * Author: MRICO
 */ 

#include "Main/Executor.h"
#include "UserInput/Command.h"

Executor executor;

Command* serialCommand = Command::getInstance();

void setup()
{
	Serial.begin(500000);
	
	executor.init();
	
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
	executor.execution();
	
	serialCommand->listenForCommands();
}
