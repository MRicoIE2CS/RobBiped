/*
 * SimpleSquatsCommandedByPotentiometer.cpp
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

/*
 * Example -> SimpleSquatsCommandedByPotentiometer.cpp
 *
 * This file is an example (copy-pasted from Executor.cpp) where motion of the legs
 * is controlled using one potentiometer.
 * Potentiometer signal regulates the length of the legs, with a fixed vertical
 * setpoint angle for the legs motion, and compensates the angle of the torso to
 * remain vertical, using the rotation matrix of the final effector.
 * Torso posture and foot roll controllers can act in parallel.
 *
 * This example does not make use of any state machine class, as the steps to
 * reproduce are commanded via serial commands.
 * The commands, at the time of development are:
 * - "init"
 * - "squats.on"
 * - "servo.pwr" > *At the time of execution of this command, make sure that the
 * potentiometer is on the extreme returning maximum signal value, and that the
 * robot is in stand up position. The servos could perform a big speed movement
 * depending on the initial position of the robot and the potentiometer.
 * After this last command, the robot will perform a "squat" movements while the
 * potentiometer is turned from one side to the other.
 */

#ifdef false

#include "Executor.h"

#include <iostream>
#include <vector>

#include "../Utils/Kinematics/ForwardKinematics.h"
#include "../Utils/Kinematics/GeometricInverseKinematics.h"
#include "../Utils/LinearAlgebra/ArduinoEigenDense.h"

using Eigen::IOFormat;
using Eigen::Matrix2d;

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
	
	// This task will be computed each time a new gyroscope/accelerometer measurement is performed.
	// For that reason, there is no need to set an execution period for it.
	torso_posture_controller_.init();
	torso_posture_controller_.set_setpoint_rad(torso_setpoint_);

	// This task will be computed each time a new foot force sensors measurement is performed.
	// For that reason, there is no need to set an execution period for it.
	left_foot_roll_centering_controller_.init();
	left_foot_roll_centering_controller_.set_setpoint_rad(zmp_lateral_deviation_setpoint_);
	right_foot_roll_centering_controller_.init();
	right_foot_roll_centering_controller_.set_setpoint_rad(zmp_lateral_deviation_setpoint_);

	// This task could be updated in response to a request from other task, using `should_be_updated` member,
	// or every execution period of the task could be checked if it needs to be updated.
	// The signal generation will be updated each time a new gyroscope/accelerometer measure is taken.
	servo_updater_.set_execution_period(I_PeriodicTask::execType::inMillis,2);
	servo_updater_.init();

	// Signal generator to generate the squats periodic movement.
	squats_unitary_cycle_generator_.configure_signal(SignalGenerator::SignalType::sine, 10000, 0.5, 0.5, HALF_PI);

	// END TASKS CONFIGURATION

	// Get Command singleton instance
	command_ = Command::get_instance();

	initialize_servo_setpoints();
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
	// Squats commands
	if (command_->commands.squats_on)
	{
		squats_on = true;
Serial.println("squats_on::: ");
		command_->commands.squats_on = false;
	}
	if (command_->commands.squats_off)
	{
		squats_on = false;
Serial.println("squats_off::: ");
		command_->commands.squats_off = false;
	}
	//__________________

	// If sensors have been updated, control can be computed.
	if (gyroscope_accelerometer_manager_.has_been_updated)
	{
		// Squats generation:::
		// With the signal generator, direct kinematics are applied to form a squat.
		double sine_unitary_signal = 0.0;
		double potentiometer_value = 0.0;
		if (squats_on)
		{
			if (squats_first_time)
			{
				Serial.println("squats_first time::: ");
				squats_unitary_cycle_generator_.init();
				squats_first_time = false;
			}

			//sine_unitary_signal = squats_unitary_cycle_generator_.generate_trajectory();
			potentiometer_value = user_input_.get_analog_value(UserInput::AnalogInputList::potentiometer1) / 4095.0;

			if (command_->commands.squats_debug_on)
			{
				Serial.println("Squats signal::: " + (String)potentiometer_value);

				if (command_->commands.squats_debug_off)
				{
					command_->commands.squats_debug_on = false;
					command_->commands.squats_debug_off = false;
				}
			}

		}
		else squats_first_time = true;

		IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

		// Desired leg length
		double desired_leg_lenght = 75 + (142-75) * potentiometer_value;
		// Lengths definition
		const double thigh = 79;
		const double calf = 63;

		// INVERSE KINEMATICS

		double forward_angle_rad = 0.0;
		double lateral_angle_rad = 0.0;	// Not used
		Vector3d desired_position;
		Serial.println("Initial leg position: ");
		Serial.println("desired_leg_lenght: " + (String)desired_leg_lenght);
		Serial.println("forward_angle_rad: " + (String)forward_angle_rad);
		Serial.println("lateral_angle_rad: " + (String)lateral_angle_rad);

		ForwardKinematics::Geometric::get_position_from_length_and_angles(desired_leg_lenght, forward_angle_rad, lateral_angle_rad, desired_position);
		Serial.println("Desired position: ");
		std::cout << desired_position.format(CleanFmt) << std::endl;

		Vector2d links_lengths;
		links_lengths << calf, thigh;
		double target_angle_rad_1;
		double target_angle_rad_2;
		bool ret_val = InverseKinematics::Geometric::sagittal_two_links_inverse_kinematics(desired_position, links_lengths, target_angle_rad_1, target_angle_rad_2);
		Serial.println("target joint angles: " + (String)(target_angle_rad_1 * RAD_TO_DEG) + ", " + (String)(target_angle_rad_2 * RAD_TO_DEG));

		// OBTAIN TRTANSFORMATION MATRIX to know the orientation of final effector

		// Denavit-Hartenberg table
		std::vector<Vector4d> DH_table;
		Vector4d DH_row_1;
		DH_row_1 << target_angle_rad_1, 0, calf, 0;
		Vector4d DH_row_2;
		DH_row_2 << target_angle_rad_2, 0, thigh, 0;
		DH_table.push_back(DH_row_1);
		DH_table.push_back(DH_row_2);

		Matrix4d TM;

		if (ret_val) ForwardKinematics::DenavitHartenberg::get_overall_TM_from_DH_table(DH_table, TM);

		Serial.println("TF: ");
		std::cout << TM.format(CleanFmt) << std::endl;

		Matrix2d rotation_matrix = TM.block<2,2>(0,0);
		Serial.println("rotation_matrix: ");
		std::cout << rotation_matrix.format(CleanFmt) << std::endl;

		double angle_to_compensate_hip = 0.0;
		if (ret_val & squats_on)
		{
			angle_to_compensate_hip = atan2(TM(0,1), TM(0,0));

			// Servo setpoint assignation.
			servo_updater_.set_angle_to_servo(Configuration::JointsNames::LeftFootPitch, target_angle_rad_1);
			servo_updater_.set_angle_to_servo(Configuration::JointsNames::LeftKnee, target_angle_rad_2);
			servo_updater_.set_angle_to_servo(Configuration::JointsNames::RightFootPitch, target_angle_rad_1);
			servo_updater_.set_angle_to_servo(Configuration::JointsNames::RightKnee, target_angle_rad_2);
			servo_updater_.should_be_updated = true;
		}

		Serial.println("target joint angles: " + (String)(angle_to_compensate_hip * RAD_TO_DEG));


		// Torso upright control:::
		// Get torso pitch orientation measurement.
		double filtered_torso_pitch_angle_rad = torso_pitch_exp_filter_.filter(gyroscope_accelerometer_manager_.get_value_angle_z_pitch_rad());

		// Compute the control action for torso pitch.
		double torso_pitch_control_action = torso_posture_controller_.compute(filtered_torso_pitch_angle_rad);

		// Servo setpoint assignation.
		servo_updater_.set_angle_to_servo(Configuration::JointsNames::LeftHipPitch, torso_pitch_control_action + angle_to_compensate_hip);
		servo_updater_.set_angle_to_servo(Configuration::JointsNames::RightHipPitch, torso_pitch_control_action + angle_to_compensate_hip);
		servo_updater_.should_be_updated = true;
	}

	if (force_sensors_manager_.has_been_updated)
	{
		double left_foot_roll_centering_action = 0.0;
		if (force_sensors_manager_.is_tare_left_performed())
		{
			int16_t left_foot_zmp_lateral_deviation;
			int16_t ignored;
			force_sensors_manager_.get_values_ZMP_LeftFoot(ignored, left_foot_zmp_lateral_deviation);
			double d_left_foot_zmp_lateral_deviation = left_zmp_lateral_exp_filter_.filter(static_cast<double>(left_foot_zmp_lateral_deviation));
			left_foot_roll_centering_action = left_foot_roll_centering_controller_.compute(d_left_foot_zmp_lateral_deviation);
		}

		double right_foot_roll_centering_action = 0.0;
		if (force_sensors_manager_.is_tare_right_performed())
		{
			int16_t right_foot_zmp_lateral_deviation;
			int16_t ignored;
			force_sensors_manager_.get_values_ZMP_RightFoot(ignored, right_foot_zmp_lateral_deviation);
			double d_right_foot_zmp_lateral_deviation = right_zmp_lateral_exp_filter_.filter(static_cast<double>(right_foot_zmp_lateral_deviation));
			right_foot_roll_centering_action = right_foot_roll_centering_controller_.compute(d_right_foot_zmp_lateral_deviation);
		}

		// Servo setpoint assignation.
		servo_updater_.set_angle_to_servo(Configuration::JointsNames::LeftFootRoll, left_foot_roll_centering_action);
		servo_updater_.set_angle_to_servo(Configuration::JointsNames::RightFootRoll, right_foot_roll_centering_action);
		servo_updater_.should_be_updated = true;
	}
}

void Executor::outputs()
{
	// At the execution periodicity configured, for each servo, it is checked if it needs new setpoint.
	// If that is the case, the setpoint of that servo is sent over I2C to the PCA9685.
	if (servo_updater_.get_execution_flag())
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

#endif
