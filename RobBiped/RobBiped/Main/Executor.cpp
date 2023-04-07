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

	// Force sensors new measurement is ready every 67500us.
	// This execution period sets the period at which the measurement is checked to be ready.
	force_sensors_manager_.set_execution_period(I_PeriodicTask::execType::inMillis, 1);
	force_sensors_manager_.init();

	// Every time a new measurement of the gyroscope/accelerometer is taken, reading requires 2ms of elapsed time.
	// This task sets the calculation period of the torso posture control.
	gyroscope_accelerometer_manager_.set_execution_period(I_PeriodicTask::execType::inMillis, 10);
	gyroscope_accelerometer_manager_.init();

	// This task will depend on the update of the setpoints of the servos, which is triggered by other tasks,
	// such as the control tasks.
	servo_updater_.set_execution_period(I_PeriodicTask::execType::inMillis,5);
	servo_updater_.init();

	// END TASKS CONFIGURATION

	initialize_servo_setpoints();
	servo_updater_.should_be_updated = true;
}

void Executor::inputs()
{
	// These are flags set true for just one execution loop.
	force_sensors_manager_.has_been_updated = false;
	gyroscope_accelerometer_manager_.has_been_updated = false;

	// User input updates at a rate specified on `init()` method.
	if (user_input_.get_execution_flag()) user_input_.update();

	// Force sensors manager updates at a rate specified on `init()` method.
	// Each time, sensors must be ready. If they are not, an error flag is set.
	// Execution period should be configured bigger that the time the sensor needs, for that to not happen.
	if (force_sensors_manager_.get_execution_flag())
	{
		force_sensors_manager_.has_been_updated = force_sensors_manager_.update();
	}

	// If the force sensors manager have been updated, the gyroscope/accelerometer is read.
	// This is because force sensors need more time to update internally.
	// Also, gyro/acc requires considerable more time to be read.
	if (gyroscope_accelerometer_manager_.get_execution_flag())
	{
		gyroscope_accelerometer_manager_.has_been_updated = gyroscope_accelerometer_manager_.update();
	}
}

void Executor::main_execution()
{
	// If sensors have been updated, control can be computed.
	if (gyroscope_accelerometer_manager_.has_been_updated)
	{
		// TODO
	}
}

void Executor::outputs()
{
	// If another task has raised the flag, because new setpoints have been set.
	if (servo_updater_.should_be_updated)
	{
		// Servo setpoint command
		// This implies I2C communication, but don't worry,
		// it will send the command only for the servos whose setpoint has been updated.
		servo_updater_.update(user_input_);
		servo_updater_.should_be_updated = false;
	}
}

void Executor::execution()
{
	inputs();
	
	main_execution();
	
	outputs();
}
