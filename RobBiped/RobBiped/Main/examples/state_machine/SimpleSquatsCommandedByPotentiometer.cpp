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
 * This file is an example (copy-pasted from StateMachine.cpp) where motion of
 * the legs is controlled using one potentiometer.
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

/*
 * StateMachine.cpp
 *
 * Created: 15/04/2023 14:30:36
 *  Author: MRICO
 */

#include "Executor.h"

#include <iostream>
#include <vector>

#include "../Utils/Kinematics/ForwardKinematics.h"
#include "../Utils/Kinematics/GeometricInverseKinematics.h"
#include "../Utils/LinearAlgebra/ArduinoEigenDense.h"

using Eigen::IOFormat;
using Eigen::Matrix2d;

void Executor::state_machine_switch()
{
	read_commands();

	switch (state_number)
	{
		case 0:
			if (squats_on)
			{
				state_number++;
				break;
			}
			break;
		case 1:
			if (!squats_on)
			{
				state_number--;
				break;
			}
			break;
		case 2:
			break;
	}
}

void Executor::state_machine_execution()
{
	switch (state_number)
	{
		case 0:
			state0_execution();
			break;
		case 1:
			state1_execution();
			break;
		case 2:
			state2_execution();
			break;
	}
}

void Executor::read_commands()
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
}

void Executor::state0_execution()
{
	squats_first_time = true;
}

void Executor::state1_execution()
{
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

		// OBTAIN TRTANSFORMATION MATRIX to know the orientation of final the effector

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
		servo_updater_.set_angle_to_servo(Configuration::JointsNames::LeftHipPitch, -torso_pitch_control_action + angle_to_compensate_hip);
		servo_updater_.set_angle_to_servo(Configuration::JointsNames::RightHipPitch, -torso_pitch_control_action + angle_to_compensate_hip);
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

void Executor::state2_execution()
{
	
}

#endif
