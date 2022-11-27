/*
 * SerialCommand.cpp
 *
 * Created: 25/11/2022 22:05:27
 *  Author: MRICO
 */ 

#include "SerialCommand.h"

SerialCommand* SerialCommand::_pointer = nullptr;

void SerialCommand::listenForCommands()
{
	if(Serial.available()){
		String command = Serial.readStringUntil('\n');
		
		if (command.equals("init")){
			commands.init = true;
		}
		else if (command.equals("gyroacc_calibrate_on")){
			commands.gyroacc_calibrate_on = true;
		}
		else if (command.equals("gyroacc_calibrate_off")){
			commands.gyroacc_calibrate_off = true;
		}
		else if (command.equals("gyroacc_debug_on")){
			commands.gyroacc_debug_on = true;
		}
		else if (command.equals("gyroacc_debug_off")){
			commands.gyroacc_debug_off = true;
		}
		else if (command.equals("off"))
		{
		}
	}
}

SerialCommand* SerialCommand::getInstance()
{
	if(_pointer==nullptr){
		_pointer = new SerialCommand();
	}
	return _pointer;
}
