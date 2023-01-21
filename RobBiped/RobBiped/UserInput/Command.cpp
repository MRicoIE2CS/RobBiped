/*
 * Command.cpp
 *
 * Copyright 2023 Mikel Rico Abajo (https://github.com/MRicoIE2CS)

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

#include "Command.h"

Command* Command::pointer_ = nullptr;

Command* Command::get_instance()
{
	if(pointer_ == nullptr){
		pointer_ = new Command();
	}
	return pointer_;
}

void Command::listen_for_commands()
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
