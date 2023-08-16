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

void Executor::read_commands()
{
	// Commands
	if (command_->commands.application_onoff && !application_on)
	{
		application_on = true;
		Serial.println("application on::: ");
		command_->commands.application_onoff = false;
	}
	if (command_->commands.application_onoff && application_on)
	{
		application_on = false;
		Serial.println("application off::: ");
		command_->commands.application_onoff = false;
	}
	if (command_->commands.get_up_down && !get_up)
	{
		get_up = true;
		Serial.println("get up::: ");
		command_->commands.get_up_down = false;
	}
	if (command_->commands.get_up_down && get_up)
	{
		get_up = false;
		Serial.println("get down::: ");
		command_->commands.get_up_down = false;
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
				state_number = 1;
				state1_first_time = true;
				state1_phase = 0;
				sin_signal.init();
				break;
			}
			if (get_up)
			{
				state_number = 2;
				state2_first_time = true;
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
			if (!application_on)
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
			if (!get_up)
			{
				state_number = 0;
				state0_first_time = true;
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
				break;
			}
			break;
		case 10:
			if (state10_finished)
			{
				//state_number++;
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
		case 10:
			state10_execution();
			break;
	}
}

void Executor::always_executes()
{
	if (gyroscope_accelerometer_manager_.has_been_updated)
	{
		// CM estimation computation:::
		global_kinematics_.compute_CoM_location();

		// Torso upright control:::

		// Get torso pitch orientation measurement.
		double filtered_torso_pitch_angle_rad = torso_pitch_exp_filter_.filter(gyroscope_accelerometer_manager_.get_value_angle_z_pitch_rad());
		// Compute the control action for torso pitch.
		torso_upright_pitch_control_action = torso_posture_controller_.compute(filtered_torso_pitch_angle_rad);
	}

	if (force_sensors_manager_.has_been_updated)
	{
		// ZMP Measurement computation
		force_sensors_manager_.compute_global_ZMP(&global_kinematics_);

// 		double potentiometer_value = user_input_.get_analog_value(UserInput::AnalogInputList::potentiometer1) / 4095.0;
// 		// Desired leg length
// 		double local_zmp_lateral_deviation_setpoint_ = -37.0/2.0 + 37.0 * potentiometer_value;
// 		left_foot_roll_centering_controller_.set_setpoint_mm(local_zmp_lateral_deviation_setpoint_);
// 		right_foot_roll_centering_controller_.set_setpoint_mm(local_zmp_lateral_deviation_setpoint_);

// 		// Foot-roll controllers
// 		left_foot_roll_centering_action = 0.0;
// 		if (force_sensors_manager_.is_tare_left_performed())
// 		{
// 			double left_foot_zmp_lateral_deviation;
// 			double ignored;
// 			force_sensors_manager_.get_values_ZMP_LeftFoot(ignored, left_foot_zmp_lateral_deviation);
// 			double d_left_foot_zmp_lateral_deviation = - left_foot_zmp_lateral_deviation;
// 			left_foot_roll_centering_action = left_foot_xZMP_tracking_controller_.compute(d_left_foot_zmp_lateral_deviation);
// 		}
// 
// 		right_foot_roll_centering_action = 0.0;
// 		if (force_sensors_manager_.is_tare_right_performed())
// 		{
// 			double right_foot_zmp_lateral_deviation;
// 			double ignored;
// 			force_sensors_manager_.get_values_ZMP_RightFoot(ignored, right_foot_zmp_lateral_deviation);
// 			double d_right_foot_zmp_lateral_deviation = - right_foot_zmp_lateral_deviation;
// 			right_foot_roll_centering_action = right_foot_xZMP_tracking_controller_.compute(d_right_foot_zmp_lateral_deviation);
// 		}
	}
}

void Executor::state0_execution()
{
	if (state0_first_time)
	{
		state0_first_time = false;

		global_kinematics_.set_desired_hip_height(desired_hip_height_);

		double home_roll_angle = global_kinematics_.get_home_roll_angle();

		bool ret_val1 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftHipRoll, home_roll_angle);//global_kinematics_.compensate_hip_roll_angle(home_roll_angle));
		bool ret_val2 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightHipRoll, -home_roll_angle);//global_kinematics_.compensate_hip_roll_angle(home_roll_angle));
		bool ret_val3 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftFootRoll, -home_roll_angle);
		bool ret_val4 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightFootRoll, home_roll_angle);

		double home_leg_length = global_kinematics_.get_home_leg_lengths();
		home_leg_length = home_leg_length - config_.kinematics.height_hip - config_.kinematics.height_ankle;
		double ankle_pitch_angle;
		double knee_pitch_angle;
		double hip_pitch_angle;
		bool ret_val5 = global_kinematics_.get_joint_angles_for_leg_length(home_leg_length, 0.0, ankle_pitch_angle, knee_pitch_angle, hip_pitch_angle);

		ret_val1 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftFootPitch, ankle_pitch_angle);
		ret_val2 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftKnee, knee_pitch_angle);
		ret_val3 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftHipPitch, hip_pitch_angle + torso_upright_pitch_control_action);

		ret_val1 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightFootPitch, ankle_pitch_angle);
		ret_val2 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightKnee, knee_pitch_angle);
		ret_val3 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightHipPitch, hip_pitch_angle + torso_upright_pitch_control_action);
	}

	if (gyroscope_accelerometer_manager_.has_been_updated)
	{
		double home_leg_length = global_kinematics_.get_home_leg_lengths();
		home_leg_length = home_leg_length - config_.kinematics.height_hip - config_.kinematics.height_ankle;
		double ankle_pitch_angle;
		double knee_pitch_angle;
		double hip_pitch_angle;
		global_kinematics_.get_joint_angles_for_leg_length(home_leg_length, 0.0, ankle_pitch_angle, knee_pitch_angle, hip_pitch_angle);

		servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftFootPitch, ankle_pitch_angle);
		servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftKnee, knee_pitch_angle);
		servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftHipPitch, hip_pitch_angle + torso_upright_pitch_control_action);

		servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightFootPitch, ankle_pitch_angle);
		servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightKnee, knee_pitch_angle);
		servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightHipPitch, hip_pitch_angle + torso_upright_pitch_control_action);
	}

	// Robot starts in home position: All Joints' angles = 0.
	// After "servo.pwr" command, the "app.on" command starts the execution of 
	// After typing the starting command, "torso.on" command could be sent for that controller to better
	// stabilize the torso posture.
}

void Executor::state1_execution()
{
	// STATE 1: Just apply setpoints to ankle joints, and torso orientation

	if (state1_first_time)
	{
		state1_first_time = false;
		desired_hip_height_ = 280.0;
		global_kinematics_.set_desired_hip_height(desired_hip_height_);
		CM_path_y.start_trajectory();
		global_kinematics_.init_CoM_location();
	}

	if (force_sensors_manager_.has_been_updated)
	{
		// From pregenerated trajectory:
//		double value_CM_ref = CM_path_y.get_value();

// 		// From potentiometer:
// 		// Potentiometer value sets the CM setpoint in Double Support Phase, along the Y-axis
// 		double potentiometer_value1 = some_exp_filter_.filter(user_input_.get_analog_value(UserInput::AnalogInputList::potentiometer1) / 4095.0);
// 		// Desired CM position
// 		double DSP_CM_setpoint_ = right_foot_center_ + 20 + (desired_step_width_ - 40) * potentiometer_value1;
// 		Serial.println("DSP_CM_setpoint_: \t" + (String)DSP_CM_setpoint_);

		// Sinusoidal signal to obtain trajectory
		double unitary_value = sin_signal.generate_trajectory();
		//double DSP_CM_setpoint_ = right_foot_center_ + 20 + (desired_step_width_ - 40) * unitary_value;
		double DSP_CM_setpoint_ = -7.7 + (77+7.7) * unitary_value;
		
		// TODO: Ask forceSensorsManager if we have changed walking phase, and change the state machine accordingly
		//Serial.println("Touching ground? left,right: \t" + (String)force_sensors_manager_.is_left_foot_touching_ground() + "\t" + (String)force_sensors_manager_.is_right_foot_touching_ground());

		// Potentiometer value sets the CM setpoint in Double Support Phase, along the Y-axis
// 		double potentiometer_value2 = some_exp_filter_2_.filter(user_input_.get_analog_value(UserInput::AnalogInputList::potentiometer2) / 4095.0);
// 		// Desired hip height
// 		double desired_hip_height = 280 + 25 * potentiometer_value2;
// 		//Serial.println("desired_hip_height: \t" + (String)desired_hip_height);
// 		global_kinematics_.set_desired_hip_height(desired_hip_height_);
// 		// Desired step width
// 		desired_step_width_ = 90 + 70 * potentiometer_value2;
// 		global_kinematics_.set_desired_step_width(desired_step_width_);
// 		Serial.println("desired_step_width_: \t" + (String)desired_step_width_);
		
		bool retcode_compute_lateral_DSP_kinematics = global_kinematics_.compute_lateral_DSP_kinematics(DSP_CM_setpoint_);

// _____ SIGNAL RECORD
// 		Vector3d CoM_location;
// 		CoM_location = global_kinematics_.get_CoM_location();
// 		double ZMP_loc_x, ZMP_loc_y;
// 		force_sensors_manager_.get_global_ZMP(ZMP_loc_x, ZMP_loc_y);
// 		Serial.println("CoM_ZMP_x / CoM_ZMP_y: \t" + (String)CoM_location(0) + "\t" + (String)ZMP_loc_x + "\t" + (String)CoM_location(1) + "\t" + (String)ZMP_loc_y);
// _____

// 		// Computation of global coordinates of the ZMP
// 		double ZMP_x_mm;
// 		double ZMP_y_mm;
// 		force_sensors_manager_.get_global_ZMP(ZMP_x_mm, ZMP_y_mm);
// 		Serial.println("ZMP_x,y: \t" + (String)ZMP_x_mm + "\t" + (String)ZMP_y_mm);
		
		double left_roll_angle;
		double right_roll_angle;
		global_kinematics_.get_computed_angles(left_roll_angle, right_roll_angle);
		
		// Joint setpoint assignation.
		bool ret_val1 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftHipRoll, left_roll_angle);//global_kinematics_.compensate_hip_roll_angle(left_roll_angle));
		bool ret_val2 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightHipRoll, -right_roll_angle);//global_kinematics_.compensate_hip_roll_angle(right_roll_angle));
		bool ret_val3 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftFootRoll, -left_roll_angle);
		bool ret_val4 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightFootRoll, right_roll_angle);
		
		double left_leg_length;
		double right_leg_length;
		global_kinematics_.get_computed_leg_lengths(left_leg_length, right_leg_length);

		left_leg_length = left_leg_length - config_.kinematics.height_hip - config_.kinematics.height_ankle;
		double ankle_pitch_angle;
		double knee_pitch_angle;
		double hip_pitch_angle;
		global_kinematics_.get_joint_angles_for_leg_length(left_leg_length, 0.0, ankle_pitch_angle, knee_pitch_angle, hip_pitch_angle);
		
		ret_val1 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftFootPitch, ankle_pitch_angle);
		ret_val2 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftKnee, knee_pitch_angle);
		ret_val3 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftHipPitch, hip_pitch_angle + torso_upright_pitch_control_action);
		
		right_leg_length = right_leg_length - config_.kinematics.height_hip - config_.kinematics.height_ankle;
		global_kinematics_.get_joint_angles_for_leg_length(right_leg_length, 0.0, ankle_pitch_angle, knee_pitch_angle, hip_pitch_angle);

		ret_val1 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightFootPitch, ankle_pitch_angle);
		ret_val2 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightKnee, knee_pitch_angle);
		ret_val3 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightHipPitch, hip_pitch_angle + torso_upright_pitch_control_action);
	}
}

void Executor::state2_execution()
{
	// STATE 2: 
	
	if (state2_first_time)
	{
		state2_first_time = false;

		
	}

	if (force_sensors_manager_.has_been_updated)
	{
		// Potentiometer value sets the CM setpoint in Double Support Phase, along the Y-axis
		double potentiometer_value = some_exp_filter_.filter(user_input_.get_analog_value(UserInput::AnalogInputList::potentiometer1) / 4095.0);
		// Desired hip height
		double desired_hip_height = 280 + 25 * potentiometer_value;
		//Serial.println("desired_hip_height: \t" + (String)desired_hip_height);

		global_kinematics_.set_desired_hip_height(desired_hip_height);
		
		double home_roll_angle = 0.0;

		bool ret_val1 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftHipRoll, home_roll_angle);
		bool ret_val2 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightHipRoll, -home_roll_angle);
		bool ret_val3 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftFootRoll, -home_roll_angle);
		bool ret_val4 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightFootRoll, home_roll_angle);

		double leg_length = desired_hip_height - config_.kinematics.height_hip - config_.kinematics.height_ankle - config_.kinematics.height_foot;
		double ankle_pitch_angle;
		double knee_pitch_angle;
		double hip_pitch_angle;
		bool ret_val5 = global_kinematics_.get_joint_angles_for_leg_length(leg_length, 0.0, ankle_pitch_angle, knee_pitch_angle, hip_pitch_angle);

		ret_val1 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftFootPitch, ankle_pitch_angle);
		ret_val2 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftKnee, knee_pitch_angle);
		ret_val3 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftHipPitch, hip_pitch_angle + torso_upright_pitch_control_action);

		ret_val1 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightFootPitch, ankle_pitch_angle);
		ret_val2 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightKnee, knee_pitch_angle);
		ret_val3 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightHipPitch, hip_pitch_angle + torso_upright_pitch_control_action);

		// Computation of global coordinates of the ZMP
// 		force_sensors_manager_.compute_global_ZMP(&global_kinematics_);
// 		int16_t ZMP_x_mm;
// 		int16_t ZMP_y_mm;
// 		force_sensors_manager_.get_global_ZMP(ZMP_x_mm, ZMP_y_mm);
// 		Serial.println("ZMP_x,y: \t" + (String)ZMP_x_mm + "\t" + (String)ZMP_y_mm);
	}
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

void Executor::state10_execution()
{
}
