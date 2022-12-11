/*
 * Command.h
 *
 * Created: 24/11/2022 21:56:52
 *  Author: MRICO
 */ 


#ifndef _COMMAND_h
#define _COMMAND_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif


struct Command
{
	static Command* getInstance();

	void listenForCommands();
	
	struct CommandsList
	{
		bool init = false;
		bool servo_onoff_toggle = false;
		bool servo_selection_button_emulation = false;
		bool force_tare_left = false;
		bool force_tare_right = false;
		bool force_debug_on = false;
		bool force_debug_off = false;
		bool zmp_debug_on = false;
		bool zmp_debug_off = false;
		bool gyroacc_calibrate_on = false;
		bool gyroacc_calibrate_off = false;
		bool gyroacc_debug_on = false;
		bool gyroacc_debug_off = false;
	}commands;
	
private:

	Command(){};
	Command(const Command&); // Disabling copy-ctor
	Command& operator=(const Command&);

	static Command* _pointer;
};

#endif
