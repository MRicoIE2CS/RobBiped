/*
 * StateMachine.h
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

#include <iostream>
#include <vector>

#include "../Utils/Kinematics/ForwardKinematics.h"
#include "../Utils/Kinematics/GeometricInverseKinematics.h"
#include "../Utils/Kinematics/InverseKinematicsBlocks/LegLength_IK.h"
#include "../Utils/LinearAlgebra/ArduinoEigenDense.h"

using Eigen::IOFormat;

void Executor::read_commands()
{
	// Commands
}

void Executor::state_machine_switch()
{
	read_commands();

	switch (state_number)
	{
		case 0:
			if (command_->commands.application_toggle)
			{
				state_number = 1;
				state1_first_time = true;
				state1_phase = 0;
				break;
			}
			if (command_->commands.get_up_toggle)
			{
				state_number = 2;
				state2_first_time = true;
				break;
			}
			if (command_->commands.test_x_balance_toggle)
			{
				state_number = 10;
				state10_first_time = true;
				break;
			}
			if (command_->commands.test_y_offline_tracking_toggle)
			{
				state_number = 20;
				state20_first_time = true;
				break;
			}
			if (command_->commands.walk)
			{
				state_number = 30;
				state30_first_time = true;
				state30_not_yet_lifted_a_foot = true;
				global_kinematics_.force_current_walking_phase(GlobalKinematics::WalkingPhase::DSP_left);
				break;
			}
			break;
		case 1:
			if (state1_phase && state1_finished)
			{
// 				state_number++;
// 				waiting_first_time_ = true;
				break;
			}
			if (!command_->commands.application_toggle)
			{
				state_number = 0;
				state0_first_time = true;
				break;
			}
			break;
		case 2:
			if (!waiting_first_time_ && state2_finished)
			{
// 				state_number++;
// 				state3_phase = 0;
				break;
			}
			if (!command_->commands.get_up_toggle)
			{
				state_number = 0;
				state0_first_time = true;
				break;
			}
			break;
		case 10:
			if (state10_finished)
			{
				//state_number++;
				break;
			}
			if (!command_->commands.test_x_balance_toggle)
			{
				state_number = 0;
				state0_first_time = true;
				break;
			}
			break;
		case 20:
			if (state20_finished)
			{
				//state_number++;
				break;
			}
			if (!command_->commands.test_y_offline_tracking_toggle)
			{
				state_number = 0;
				state0_first_time = true;
				break;
			}
			break;
		case 30:
			if (state30_finished)
			{
				//state_number++;
				break;
			}
			if (!command_->commands.walk)
			{
				state_number = 0;
				state0_first_time = true;
				state30_not_yet_lifted_a_foot = true;
				global_kinematics_.force_current_walking_phase(GlobalKinematics::WalkingPhase::DSP_left);
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
		case 10:
			state10_execution();
			break;
		case 20:
			state20_execution();
			break;
		case 30:
			state30_execution();
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
		// ZMP Measurement computation:::
		force_sensors_manager_.compute_global_ZMP(&global_kinematics_);

		if (30 == state_number)
		{
			// Walking phases status is linked to the force sensors update
			global_kinematics_.check_walking_phase();
		}
	}
	
	// CM estimation computation:::
	if (gyroscope_accelerometer_manager_.has_been_updated || force_sensors_manager_.has_been_updated) global_kinematics_.compute_CoM_location();
}

void Executor::state0_execution()
{
	// Home state: Use this state to prepare the posture of the body to any application that will be used
	// Now you can move the CM in DSP over to any "reachable" position along Y-axis
	// and start controllers as "torso.on" and "zmptrx" "zmptry"

	if (state0_first_time)
	{
		state0_first_time = false;

		global_kinematics_.set_desired_hip_height(desired_hip_height_);
		global_kinematics_.set_desired_step_width(desired_step_width_);
		
		global_kinematics_.init_CoM_location();
		// Set offline reference tracking for Y-axis
		cm_tracking_controller_.set_mode(Control::CMTracking::Mode::OnlineReference, Control::CMTracking::Mode::OnlineReference);
		// And load the trajectories
		cm_tracking_controller_.init();
		// X-axis keeps balance of the CM over the X-axis
		cm_tracking_controller_.set_CM_x_online_reference(0.0);
	}

	if (force_sensors_manager_.has_been_updated)
	{
		// Potentiometer value sets the CM setpoint in Double Support Phase, along the Y-axis
		double potentiometer_value1 = some_exp_filter_.filter(user_input_.get_analog_value(UserInput::AnalogInputList::potentiometer1) / 4095.0);
		// Desired CM position
		double DSP_CM_setpoint_ = right_foot_center_ + (desired_step_width_) * potentiometer_value1;

		// Compute DSP kinematics
		bool retcode_compute_lateral_DSP_kinematics = global_kinematics_.compute_lateral_DSP_kinematics(DSP_CM_setpoint_);

		// Get roll angle setpoints from DSP kinematics
		double left_roll_angle;
		double right_roll_angle;
		global_kinematics_.get_computed_angles(left_roll_angle, right_roll_angle);

		// Get desired leg lengths computed from DSP kinematics
		double left_leg_length;
		double right_leg_length;
		global_kinematics_.get_computed_leg_lengths(left_leg_length, right_leg_length);

		// Get left leg's pitch angle setpoints from DSP kinematics
		left_leg_length = left_leg_length - config_.kinematics.height_hip - config_.kinematics.height_ankle;
		double left_ankle_pitch_angle, right_ankle_pitch_angle;
		double left_knee_pitch_angle, right_knee_pitch_angle;
		double left_hip_pitch_angle, right_hip_pitch_angle;
		global_kinematics_.get_joint_angles_for_prismatic_length(left_leg_length, 0.0, left_ankle_pitch_angle, left_knee_pitch_angle, left_hip_pitch_angle);

		// Get right leg's pitch angle setpoints from DSP kinematics
		right_leg_length = right_leg_length - config_.kinematics.height_hip - config_.kinematics.height_ankle;
		global_kinematics_.get_joint_angles_for_prismatic_length(right_leg_length, 0.0, right_ankle_pitch_angle, right_knee_pitch_angle, right_hip_pitch_angle);

		// Compute CM tracking control: The output is the ZMP setpoint
		Vector2d ZMP_ref_xy = cm_tracking_controller_.compute_ZMP_setpoint();
		
// // This commented lines avoid the CM stabilization computation, and allows setting a ZMP reference to track from the potentiometer2 signal.
// // Potentiometer value sets the ZMP setpoint in Double Support Phase, along the X-axis
// double potentiometer_value2 = some_exp_filter_2_.filter(user_input_.get_analog_value(UserInput::AnalogInputList::potentiometer2) / 4095.0);
// // Desired CM position
// ZMP_ref_xy(0) = -config_.force_sensors.location_mm.frontBack_separation/2.0 + config_.force_sensors.location_mm.frontBack_separation * potentiometer_value2;
// //ZMP_ref_xy(1) = -config_.force_sensors.location_mm.leftRight_separation/2.0 + config_.force_sensors.location_mm.leftRight_separation * potentiometer_value2;

		// Apply new ZMP setpoint for X-axis ZMP tracking controllers
		left_foot_ZMP_tracking_controller_.set_setpoint_x_mm(ZMP_ref_xy(0));
		right_foot_ZMP_tracking_controller_.set_setpoint_x_mm(ZMP_ref_xy(0));

		// Apply new ZMP setpoint for X-axis ZMP tracking controllers
		left_foot_ZMP_tracking_controller_.set_setpoint_y_mm(ZMP_ref_xy(1));
		right_foot_ZMP_tracking_controller_.set_setpoint_y_mm(ZMP_ref_xy(1));

		// Get local ZMP measurements
		Vector2d current_left_ZMP = force_sensors_manager_.get_values_ZMP_LeftFoot();
		Vector2d current_right_ZMP = force_sensors_manager_.get_values_ZMP_RightFoot();

		// Compute ZMP tracking control: The output is the increment no the setpoint angles for ankle joints
		Vector2d left_foot_ZMP_tracking_action = left_foot_ZMP_tracking_controller_.compute(current_left_ZMP(0), current_left_ZMP(1));
		Vector2d right_foot_ZMP_tracking_action = right_foot_ZMP_tracking_controller_.compute(current_right_ZMP(0), current_right_ZMP(1));

		// Apply roll angle setpoints
		double compensated_left_roll_angle = global_kinematics_.compensate_hip_roll_angle(left_roll_angle, false);
		double compensated_right_roll_angle = global_kinematics_.compensate_hip_roll_angle(-right_roll_angle, true);
		bool ret_val1 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftHipRoll, compensated_left_roll_angle);
		bool ret_val2 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightHipRoll, compensated_right_roll_angle);
		bool ret_val3 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftFootRoll, -left_roll_angle + left_foot_ZMP_tracking_action(1));
		bool ret_val4 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightFootRoll, right_roll_angle + right_foot_ZMP_tracking_action(1));

		// Apply left leg's pitch angle setpoints
		ret_val1 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftFootPitch, left_ankle_pitch_angle + left_foot_ZMP_tracking_action(0));
		ret_val2 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftKnee, left_knee_pitch_angle);
		ret_val3 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftHipPitch, left_hip_pitch_angle + torso_upright_pitch_control_action);

		// Apply right leg's pitch angle setpoints
		ret_val1 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightFootPitch, right_ankle_pitch_angle + right_foot_ZMP_tracking_action(0));
		ret_val2 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightKnee, right_knee_pitch_angle);
		ret_val3 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightHipPitch, right_hip_pitch_angle + torso_upright_pitch_control_action);
	}
}
