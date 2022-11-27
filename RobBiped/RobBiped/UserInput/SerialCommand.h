/*
 * SerialCommand.h
 *
 * Created: 24/11/2022 21:56:52
 *  Author: MRICO
 */ 


#ifndef _SERIALCOMMAND_h
#define _SERIALCOMMAND_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif


struct SerialCommand
{
	static SerialCommand* getInstance();

	void listenForCommands();
	
	struct CommandsList
	{
		bool init = false;
		bool gyroacc_calibrate_on = false;
		bool gyroacc_calibrate_off = false;
		bool gyroacc_debug_on = false;
		bool gyroacc_debug_off = false;
	}commands;
	
private:

	SerialCommand(){};
	SerialCommand(const SerialCommand&); // Disabling copy-ctor
	SerialCommand& operator=(const SerialCommand&);

	static SerialCommand* _pointer;
};

#endif
