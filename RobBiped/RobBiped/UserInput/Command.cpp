/*
 * Command.cpp
 *
 * Created: 25/11/2022 22:05:27
 *  Author: MRICO
 */ 

#include "Command.h"

Command* Command::_pointer = nullptr;

Command* Command::getInstance()
{
	if(_pointer == nullptr){
		_pointer = new Command();
	}
	return _pointer;
}

void Command::listenForCommands()
{
	if(Serial.available()){
		String command = Serial.readStringUntil('\n');
		
		if (command.equals("init")){
			commands.init = true;
		}
		else if (command.equals("servo.pwr")){
			commands.servo_onoff_toggle = true;
		}
		else if (command.equals("servo.sel")){
			commands.servo_selection_button_emulation = true;
		}
		else if (command.equals("force.tar.left")){
			commands.force_tare_left = true;
		}
		else if (command.equals("force.tar.right")){
			commands.force_tare_right = true;
		}
		else if (command.equals("force.tar.all")){
			commands.force_tare_left = true;
			commands.force_tare_right = true;
		}
		else if (command.equals("force.deb.on")){
			commands.force_debug_on = true;
		}
		else if (command.equals("force.deb.off")){
			commands.force_debug_off = true;
		}
		else if (command.equals("zmp.deb.on")){
			commands.zmp_debug_on = true;
		}
		else if (command.equals("zmp.deb.off")){
			commands.zmp_debug_off = true;
		}
		else if (command.equals("gyro.cal.on")){
			commands.gyroacc_calibrate_on = true;
		}
		else if (command.equals("gyro.cal.off")){
			commands.gyroacc_calibrate_off = true;
		}
		else if (command.equals("gyro.deb.on")){
			commands.gyroacc_debug_on = true;
		}
		else if (command.equals("gyro.deb.off")){
			commands.gyroacc_debug_off = true;
		}
		else if (command.equals("off"))
		{
		}
	}
}