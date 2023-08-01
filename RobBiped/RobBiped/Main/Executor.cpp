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
	force_sensors_manager_.set_execution_period(I_PeriodicTask::execType::inMicros, 250);
	force_sensors_manager_.init();

	// Every time a new measurement of the gyroscope/accelerometer is taken, reading requires 2ms of elapsed time.
	// This task sets the calculation period of the torso posture control.
	gyroscope_accelerometer_manager_.set_execution_period(I_PeriodicTask::execType::inMillis, 9);
	gyroscope_accelerometer_manager_.init();
	
	// This task will be computed each time a new gyroscope/accelerometer measurement is performed.
	// For that reason, there is no need to set an execution period for it.
	torso_posture_controller_.init();
	torso_posture_controller_.set_setpoint_rad(torso_setpoint_);

	// This task will be computed each time a new foot force sensors measurement is performed.
	// For that reason, there is no need to set an execution period for it.
	left_foot_roll_centering_controller_.init();
	left_foot_roll_centering_controller_.set_setpoint_mm(local_zmp_lateral_deviation_setpoint_);
	right_foot_roll_centering_controller_.init();
	right_foot_roll_centering_controller_.set_setpoint_mm(local_zmp_lateral_deviation_setpoint_);

	// This task could be updated in response to a request from other task, using `should_be_updated` member,
	// or every execution period of the task could be checked if it needs to be updated.
	// The signal generation will be updated each time a new gyroscope/accelerometer measure is taken.
	servo_updater_.set_execution_period(I_PeriodicTask::execType::inMillis, 2);
	servo_updater_.init();

	// Initialize application specific tasks and objects
	initialize_application();

	// END TASKS CONFIGURATION

	// Get Command singleton instance
	command_ = Command::get_instance();

	initialize_servo_setpoints();
}

void Executor::initialize_application()
{
	// Kinematics will be updated each time the sensors are updated, so task timing configuration is not needed.
	global_kinematics_.init(right_foot_center_, initial_phase_, desired_hip_height_, desired_step_width_);

	some_exp_filter_.set_time_constant(250);

	sin_signal.configure_signal(SignalGenerator::SignalType::sine, 2000, 0.5, 0.5, 0);
}

void Executor::inputs()
{
	// These are flags set true for just one execution loop.
	force_sensors_manager_.has_been_updated = false;
	gyroscope_accelerometer_manager_.has_been_updated = false;

	// User input updates at a rate specified on `init()` method.
	if (user_input_.get_execution_flag()) 
	{
		user_input_.update();
		user_input_.has_been_updated = true;
	}

	// Force sensors manager updates at a rate specified on `init()` method.
	if (force_sensors_manager_.get_execution_flag())
	{
		force_sensors_manager_.has_been_updated = force_sensors_manager_.update();
	}

	// The synchronization of gyro/acc sensor is dependent on force sensors execution.
	// This is because force sensors need more time to update internally (new data available every ~11-13ms).
	// Also, gyro/acc requires considerable more time to be read (2ms elapsed time when reading data).
	// So, gyro/acc will be read 9ms after the force sensors reading, spending 2ms elapsed, then force sensors will be read again,
	// triggering the execution of the application's control scheme. -> Cycle of the same period of the force sensors measurement.
	if(force_sensors_manager_.has_been_updated)
	{
		// Gyro/acc sensor is read, counting time after force sensors update.
		gyroscope_accelerometer_manager_.reset_timer();
	}
	if (gyroscope_accelerometer_manager_.get_execution_flag())
	{
		gyroscope_accelerometer_manager_.has_been_updated = gyroscope_accelerometer_manager_.update();
	}
}

void Executor::main_execution()
{
	if (servo_updater_.get_current_state() != JointsManager::State::calibrating)
	{
		state_machine_switch();

		state_machine_execution();
	}
}

void Executor::outputs()
{
	// At the execution periodicity configured, for each servo, it is checked if it needs new setpoint.
	// If that is the case, the setpoint of that servo is sent over I2C to the PCA9685.
	if (servo_updater_.get_execution_flag() || servo_updater_.should_be_updated)
	{
		servo_updater_.should_be_updated = false;
		// Servo setpoint command
		// This implies I2C communication, but don't worry,
		// it will send the command only for the servos whose setpoint has been updated.
		servo_updater_.update(user_input_);
	}
}

void Executor::execution()
{
	inputs();
	
	main_execution();
	
	outputs();
}
