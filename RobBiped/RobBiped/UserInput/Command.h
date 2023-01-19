/*
 * Command.h
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


#ifndef _COMMAND_h
#define _COMMAND_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif


struct Command
{
	static Command* get_instance();

	void listen_for_commands();
	
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

	static Command* pointer_;
};

#endif
