/*
 * Executor.cpp
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

#include "Executor.h"

void Executor::init()
{
	associations();
	
	//______TASKS CONFIGURATION_____//
	
	user_input_.set_execution_period(I_PeriodicTask::execType::inMillis, 5);
	
	servo_updater_.set_execution_period(I_PeriodicTask::execType::inMillis,20);	
	servo_updater_.init();
	
	force_sensors_manager_.set_execution_period(I_PeriodicTask::execType::inMillis, 5);
	force_sensors_manager_.init();
	
	gyroscope_accelerometer_manager_.set_execution_period(I_PeriodicTask::execType::inMillis, 4);
	gyroscope_accelerometer_manager_.init();
	
	// END TASKS CONFIGURATION
}

void Executor::inputs()
{
	if (user_input_.get_execution_flag()) user_input_.update();
	
	if (gyroscope_accelerometer_manager_.get_execution_flag())
	{
		bool updated = gyroscope_accelerometer_manager_.update();
	}
	
	if (force_sensors_manager_.get_execution_flag())
	{
		bool updated = force_sensors_manager_.update();
	}
}

void Executor::main_execution()
{
	
	
	
}

void Executor::outputs()
{
	if (servo_updater_.get_execution_flag())
	{
// 		// INIT Example of using potentiometer to command servos
// 		uint16_t pot1Val = userInput.getAnalogValue(UserInput::AnalogInputList::potentiometer2);
// 		
// 		double readingAngle_0;
// 		double max_val = 0.2;
// 		double min_val = -0.2;
// 		double ampl = max_val - min_val;
// 		if (pot1Val < 1000){
// 			readingAngle_0 = 0;
// 		}
// 		else if (pot1Val > 3000){
// 			readingAngle_0 = ampl;
// 		}
// 		else {
// 			readingAngle_0 = (double)(pot1Val - 1000) / (double)2000 * ampl;
// 		}
// 		double nextAngle_0 = readingAngle_0 - ampl / 2;
		double nextAngle_0 = 0.0;
		// END Example
		
		// Servo setpoint assignation
		servo_updater_.set_angle_to_servo(0,nextAngle_0);
		servo_updater_.set_angle_to_servo(1,nextAngle_0);
		servo_updater_.set_angle_to_servo(2,nextAngle_0);
		servo_updater_.set_angle_to_servo(3,nextAngle_0);
		servo_updater_.set_angle_to_servo(4,nextAngle_0);
		servo_updater_.set_angle_to_servo(5,nextAngle_0);
		servo_updater_.set_angle_to_servo(6,nextAngle_0);
		servo_updater_.set_angle_to_servo(7,nextAngle_0);
		servo_updater_.set_angle_to_servo(8,nextAngle_0);
		servo_updater_.set_angle_to_servo(9,nextAngle_0);
		servo_updater_.set_angle_to_servo(10,nextAngle_0);
		servo_updater_.set_angle_to_servo(11,nextAngle_0);
		servo_updater_.set_angle_to_servo(12,nextAngle_0);
		servo_updater_.set_angle_to_servo(13,nextAngle_0);
		servo_updater_.set_angle_to_servo(14,nextAngle_0);
		servo_updater_.set_angle_to_servo(15,nextAngle_0);
		
		// Servo setpoint command
		servo_updater_.update(user_input_);
	}
}

void Executor::execution()
{
	inputs();
	
	main_execution();
	
	outputs();
}