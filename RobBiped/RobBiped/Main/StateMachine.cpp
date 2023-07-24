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
#include "../Utils/Kinematics/InverseKinematicsBlocks/LegLength_IK.h"
#include "../Utils/LinearAlgebra/ArduinoEigenDense.h"

using Eigen::IOFormat;
using Eigen::Matrix2d;

void Executor::read_commands()
{
	// Commands
	if (command_->commands.application_on)
	{
		application_on = true;
		Serial.println("application on::: ");
		command_->commands.application_on = false;
	}
}

void Executor::state_machine_switch()
{
	read_commands();

	switch (state_number)
	{
		case 0:
			if (application_on)
			{
				state_number++;
				state1_phase = 0;
				break;
			}
			break;
		case 1:
			if (state1_phase && state1_finished)
			{
				state_number++;
				waiting_first_time_ = true;
				break;
			}
			break;
		case 2:
			if (!waiting_first_time_ && state2_finished)
			{
				state_number++;
				state3_phase = 0;
				break;
			}
			break;
		case 3:
			if (state3_phase && state3_finished)
			{
				state_number++;
				state4_phase = 0;
				break;
			}
			break;
		case 4:
			if (state4_phase && state4_finished)
			{
				state_number++;
				waiting_first_time_ = true;
				break;
			}
			break;
		case 5:
			if (!waiting_first_time_ && state5_finished)
			{
				state_number++;
				state6_phase = 0;
				break;
			}
			break;
		case 6:
			if (state6_phase && state6_finished)
			{
				state_number++;
				state7_phase = 0;
				break;
			}
			break;
		case 7:
			if (state7_finished)
			{
				state_number++;
				break;
			}
			break;
	}
}

void Executor::state_machine_execution()
{
	always_executes();

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
		case 3:
			state3_execution();
			break;
		case 4:
			state4_execution();
			break;
		case 5:
			state5_execution();
			break;
		case 6:
			state6_execution();
			break;
		case 7:
			state7_execution();
			break;
	}
}

void Executor::always_executes()
{
	if (gyroscope_accelerometer_manager_.has_been_updated)
	{
		// Torso upright control:::

		// Get torso pitch orientation measurement.
		double filtered_torso_pitch_angle_rad = torso_pitch_exp_filter_.filter(gyroscope_accelerometer_manager_.get_value_angle_z_pitch_rad());
		// Compute the control action for torso pitch.
		torso_upright_pitch_control_action = torso_posture_controller_.compute(filtered_torso_pitch_angle_rad);
	}

	if (force_sensors_manager_.has_been_updated)
	{

		left_foot_roll_centering_action = 0.0;
		if (force_sensors_manager_.is_tare_left_performed())
		{
			int16_t left_foot_zmp_lateral_deviation;
			int16_t ignored;
			force_sensors_manager_.get_values_ZMP_LeftFoot(ignored, left_foot_zmp_lateral_deviation);
			double d_left_foot_zmp_lateral_deviation = left_zmp_lateral_exp_filter_.filter(static_cast<double>(left_foot_zmp_lateral_deviation));
			left_foot_roll_centering_action = left_foot_roll_centering_controller_.compute(d_left_foot_zmp_lateral_deviation);
		}

		right_foot_roll_centering_action = 0.0;
		if (force_sensors_manager_.is_tare_right_performed())
		{
			int16_t right_foot_zmp_lateral_deviation;
			int16_t ignored;
			force_sensors_manager_.get_values_ZMP_RightFoot(ignored, right_foot_zmp_lateral_deviation);
			double d_right_foot_zmp_lateral_deviation = right_zmp_lateral_exp_filter_.filter(static_cast<double>(right_foot_zmp_lateral_deviation));
			right_foot_roll_centering_action = right_foot_roll_centering_controller_.compute(d_right_foot_zmp_lateral_deviation);
		}

		// TODO: Output from this controllers should be in the class scope, so that any state could apply it
	}
}

void Executor::state0_execution()
{
	if (!state0_first_time) state0_first_time = true;

	// Robot starts in home position: All Joints' angles = 0.
	// After "servo.pwr" command, the "" command starts the execution of 
	// After typing the starting command, "torso.on" command could be sent for that controller to better
	// stabilize the torso posture.
}

void Executor::state1_execution()
{
	// STATE 1: 

	bool trajectory_running = false;
	if (0 == state1_phase)
	{
		
		state1_phase = 1;
	}
	if (1 == state1_phase)
	{
	}
	if (2 == state1_phase)
	{
	}
	if (3 == state1_phase)
	{
	}
}

void Executor::state2_execution()
{
	// STATE 2: 
}

void Executor::state3_execution()
{
	// STATE 3: 
}

void Executor::state4_execution()
{
	// STATE 4: 
}

void Executor::state5_execution()
{
	// STATE 5: 
}

void Executor::state6_execution()
{
	// STATE 6: 
}

void Executor::state7_execution()
{
}
