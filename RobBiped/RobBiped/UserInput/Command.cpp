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
	// SERVO POWER ENABLE
		else if (command.equals("servo.pwr")){
			commands.servo_onoff_toggle = true;
		}
	// SERVO CALIBRATION
		else if (command.equals("servo.calib")){
			commands.servo_calibration_onoff_toggle = true;
		}
		else if (command.equals("servo.sel")){
			commands.servo_selection_button_emulation = true;
		}
	// ENABLE NOTIFICATIONS FOR OVERLIMIT ANGLE
		else if (command.equals("show.servo.limits")){
			if (commands.show_alarm_angle_limit) commands.show_alarm_angle_limit = false;
			else commands.show_alarm_angle_limit = true;
		}
	// FORCE SENSORS MANAGEMENT
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
		else if (command.equals("force.tar.auto")){
			commands.force_auto_tare = true;
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
	// GYROSCOPE/ACCELEROMETER MANAGEMENT
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
	// TORSO POSTURE CONTROLLER
		else if (command.equals("torso.deb.on")){
			commands.torso_posture_debug_on = true;
		}
		else if (command.equals("torso.deb.off")){
			commands.torso_posture_debug_off = true;
		}
		else if (command.equals("torso.on")){
			commands.torso_posture_on = true;
		}
		else if (command.equals("torso.off")){
			commands.torso_posture_off = true;
		}
	// ZMP TRACKING CONTROLLER
		else if (command.equals("zmptr.deb")){
			if (commands.zmp_tracking_deb_toggle) commands.zmp_tracking_deb_toggle = false;
			else commands.zmp_tracking_deb_toggle = true;
		}
		else if (command.equals("zmptrx")){
			if (commands.zmp_xtracking_toggle) commands.zmp_xtracking_toggle = false;
			else commands.zmp_xtracking_toggle = true;
		}
		else if (command.equals("zmptry")){
			if (commands.zmp_ytracking_toggle) commands.zmp_ytracking_toggle = false;
			else commands.zmp_ytracking_toggle = true;
		}
	// APPLICATION TOGGLE
		else if (command.equals("app.tgl")){
			if (commands.application_toggle) commands.application_toggle = false;
			else commands.application_toggle = true;
		}
	// GET UP AND DOWN ROUTINE
		else if (command.equals("get.up")){
			if (commands.get_up_toggle) commands.get_up_toggle = false;
			else commands.get_up_toggle = true;
		}
	// X-BALANCE TEST, (state number 10)
		else if (command.equals("xbalance")){
			if (commands.test_x_balance_toggle) commands.test_x_balance_toggle = false;
				else commands.test_x_balance_toggle = true;
		}
	// Y OFFLINE TRAJECTORY TRACKING TEST, (state number 20)
		else if (command.equals("ytrack")){
			if (commands.test_y_offline_tracking_toggle) commands.test_y_offline_tracking_toggle = false;
			else commands.test_y_offline_tracking_toggle = true;
		}
		else if (command.equals("off"))
		{
		}
	}
}
