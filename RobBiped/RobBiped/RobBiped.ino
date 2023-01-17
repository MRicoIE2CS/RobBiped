/*
 * RobBiped.ino
 *
 * Copyright 2023 Mikel Rico Abajo (MRicoIE2C)

 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at

 * http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
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
