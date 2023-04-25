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
	// Squats commands
	if (command_->commands.squats_on)
	{
		squats_on_ = true;
		Serial.println("squats_on::: ");
		command_->commands.squats_on = false;
	}
	if (command_->commands.squats_off)
	{
		squats_on_ = false;
		Serial.println("squats_off::: ");
		command_->commands.squats_off = false;
	}
	
	// Automatic tare commands
	if (command_->commands.force_auto_tare)
	{
		command_->commands.force_auto_tare = false;
		automatic_force_tare_on = true;
	}
}

void Executor::state_machine_switch()
{
	read_commands();

	switch (state_number)
	{
		case 0:
			if (automatic_force_tare_on)
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
				// Squats disabled!
				state_number = 10;
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
		// Feet roll control:::
		// TODO: Correct the deviation of the joint axis respect to the foot axis

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

		// TODO: Output from this controllers should be in the class scope, so that any state could apply it
	}
}

void Executor::state0_execution()
{
	if (!state0_first_time) state0_first_time = true;

	// Robot starts in home position: All Joints' angles = 0.
	// After "servo.pwr" command, the "force.tar.auto" command starts the execution of the automatic tare of the force sensors
	// located on the feet. After typing the starting command, "torso.on" command could be sent for that controller to better
	// stabilize the torso posture.
}

void Executor::state1_execution()
{
	// STATE 1: Pivot to the right foot.
	// In this state, from home position, the robot pivots to the right in order to stand on the right foot.
	// The movement is divided in three different linear movements.
	// First, a squat movement, then the pivot, and after that, a small retraction of the left leg to leave more space with the floor.

	bool trajectory_running = false;
	if (0 == state1_phase)
	{
		if (!autotare_interpolator1_.configure_trayectory(0.0, 1.0, autotare_trajectories_time_))
		while (true)
		{
			Serial.println("ERR_BAD_PARAMETER");
			delay(1000);
		}
		state1_phase = 1;
	}
	if (1 == state1_phase)
	{
		if (autotare_interpolator1_.get_execution_flag())
		{
			// TRAJECTORY UNITARY COMPUTATION
			double trajectory_unitary_vector;
			trajectory_running = autotare_interpolator1_.compute_output(trajectory_unitary_vector);
			
			// LEG LIFT DISPLACEMENT
			double calculated_leg_length = home_position_leg_length_ - autotare_legs_squat_displacement * trajectory_unitary_vector;

			//Inverse kinematics
			double forward_angle_rad = 0.0;
			double ankle_target_angle;
			double knee_target_angle;
			double hip_angle_compensation;

			bool ret_val_0 = InverseKinematics::get_leg_joints_angles_from_desired_length_and_orientation(
			calculated_leg_length, forward_angle_rad,
			config_.kinematics.height_knee_ankle, config_.kinematics.height_hip_knee,
			ankle_target_angle, knee_target_angle, hip_angle_compensation);

			// Joint setpoint assignation.
			bool ret_val1 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftFootPitch, ankle_target_angle);
			bool ret_val2 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftKnee, knee_target_angle);
			bool ret_val5 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftHipPitch, -torso_upright_pitch_control_action + hip_angle_compensation);
			bool ret_val3 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightFootPitch, ankle_target_angle);
			bool ret_val4 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightKnee, knee_target_angle);
			bool ret_val6 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightHipPitch, -torso_upright_pitch_control_action + hip_angle_compensation);
			last_hip_left_angle_compensation = hip_angle_compensation;
			last_hip_right_angle_compensation = hip_angle_compensation;

			servo_updater_.should_be_updated = true;
			
			if (!trajectory_running)
			{
				state1_phase = 2;
				autotare_interpolator1_.reset();
			}
		}
	}
	if (2 == state1_phase)
	{
		if (autotare_interpolator1_.get_execution_flag())
		{
			// TRAJECTORY UNITARY COMPUTATION
			double trajectory_unitary_vector;
			trajectory_running = autotare_interpolator1_.compute_output(trajectory_unitary_vector);

			// LATERAL DISPLACEMENT
			double lateral_displacement_angle = autotare_angle_displacement_ * trajectory_unitary_vector;

			servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftFootRoll, lateral_displacement_angle);
			servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftHipRoll, lateral_displacement_angle/4.0);
			servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightFootRoll, -lateral_displacement_angle);

			// This allows the torso upright controller to still actuate over the hip pitch joints
			bool ret_val5 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftHipPitch, -torso_upright_pitch_control_action + last_hip_left_angle_compensation);
			bool ret_val6 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightHipPitch, -torso_upright_pitch_control_action + last_hip_right_angle_compensation);

			servo_updater_.should_be_updated = true;

			if (!trajectory_running)
			{
				state1_phase = 3;
				autotare_interpolator1_.reset();
			}
		}
	}
	if (3 == state1_phase)
	{
		if (autotare_interpolator1_.get_execution_flag())
		{
			// TRAJECTORY UNITARY COMPUTATION
			double trajectory_unitary_vector;
			trajectory_running = autotare_interpolator1_.compute_output(trajectory_unitary_vector);
			
			// LEG LIFT DISPLACEMENT
			double calculated_leg_length = home_position_leg_length_ - autotare_legs_squat_displacement - autotare_leg_lift_displacement * trajectory_unitary_vector;

			//Inverse kinematics
			double forward_angle_rad = 0.0;
			double ankle_target_angle;
			double knee_target_angle;
			double hip_angle_compensation;

			bool ret_val_0 = InverseKinematics::get_leg_joints_angles_from_desired_length_and_orientation(
			calculated_leg_length, forward_angle_rad,
			config_.kinematics.height_knee_ankle, config_.kinematics.height_hip_knee,
			ankle_target_angle, knee_target_angle, hip_angle_compensation);

			// Joint setpoint assignation.
			bool ret_val1 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftFootPitch, ankle_target_angle);
			bool ret_val2 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftKnee, knee_target_angle);
			bool ret_val5 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftHipPitch, -torso_upright_pitch_control_action + hip_angle_compensation);
			bool ret_val6 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightHipPitch, -torso_upright_pitch_control_action + last_hip_right_angle_compensation);
			last_hip_left_angle_compensation = hip_angle_compensation;

			servo_updater_.should_be_updated = true;

			if (!trajectory_running)
			{
				state1_finished = true;
				autotare_interpolator1_.reset();
			}
		}
	}
}

void Executor::state2_execution()
{
	// STATE 2: Waiting.
	// The robot waits a few seconds in order to gather data for the tare of the left foot.

	if (waiting_first_time_)
	{
		waiting_first_time_ = false;
		trajectory_interpolator_.reset();
		waiting_.configure_waiting(waiting_time_);
		if (squats_going_down_) squats_going_down_ = false;
		else squats_going_down_ = true;
	}
	if (waiting_.get_execution_flag())
	{
		if (waiting_.evaluate())
		{
			// Set command to tare left foot.
			command_->commands.force_tare_left = true;
			state2_finished = true;
		}

		// Torso upright posture control is still active during waiting
		// This avoids the outgrow of the integral part of the controller
		bool ret_val5 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftHipPitch, -torso_upright_pitch_control_action + last_hip_left_angle_compensation);
		bool ret_val6 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightHipPitch, -torso_upright_pitch_control_action + last_hip_right_angle_compensation);
	}
}

void Executor::state3_execution()
{
	// STATE 3: Go from the right to home position (almost, as the squat is not reversed yet).
	// In this state, the movements of the state 1 are inversely performed.
	// First, the left leg is extended to the same length as the right one, then the pivot movement is reversed.
	// The squat movement is not reversed, as the robot will pivot to the left on the next state.

	bool trajectory_running = false;
	if (0 == state3_phase)
	{
		// Note that the interpolator goes now from 1.0 to 0.0!
		// This way, the movements are performed inversely
		// (phases have been also reversed)
		if (!autotare_interpolator1_.configure_trayectory(1.0, 0.0, autotare_trajectories_time_))
		while (true)
		{
			Serial.println("ERR_BAD_PARAMETER");
			delay(1000);
		}
		state3_phase = 1;
	}
	if (1 == state3_phase)
	{
		if (autotare_interpolator1_.get_execution_flag())
		{
			// TRAJECTORY UNITARY COMPUTATION
			double trajectory_unitary_vector;
			trajectory_running = autotare_interpolator1_.compute_output(trajectory_unitary_vector);
		
			// LEG LIFT DISPLACEMENT
			double calculated_leg_length = home_position_leg_length_ - autotare_legs_squat_displacement - autotare_leg_lift_displacement * trajectory_unitary_vector;

			//Inverse kinematics
			double forward_angle_rad = 0.0;
			double ankle_target_angle;
			double knee_target_angle;
			double hip_angle_compensation;

			bool ret_val_0 = InverseKinematics::get_leg_joints_angles_from_desired_length_and_orientation(
			calculated_leg_length, forward_angle_rad,
			config_.kinematics.height_knee_ankle, config_.kinematics.height_hip_knee,
			ankle_target_angle, knee_target_angle, hip_angle_compensation);

			// Joint setpoint assignation.
			bool ret_val1 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftFootPitch, ankle_target_angle);
			bool ret_val2 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftKnee, knee_target_angle);
			bool ret_val5 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftHipPitch, -torso_upright_pitch_control_action + hip_angle_compensation);
			bool ret_val6 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightHipPitch, -torso_upright_pitch_control_action + last_hip_right_angle_compensation);
			last_hip_left_angle_compensation = hip_angle_compensation;

			servo_updater_.should_be_updated = true;

			if (!trajectory_running)
			{
				state3_phase = 2;
				autotare_interpolator1_.reset();
			}
		}
	}
	if (2 == state3_phase)
	{
		if (autotare_interpolator1_.get_execution_flag())
		{
			// TRAJECTORY UNITARY COMPUTATION
			double trajectory_unitary_vector;
			trajectory_running = autotare_interpolator1_.compute_output(trajectory_unitary_vector);

			// LATERAL DISPLACEMENT
			double lateral_displacement_angle = autotare_angle_displacement_ * trajectory_unitary_vector;

			servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftFootRoll, lateral_displacement_angle);
			servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftHipRoll, lateral_displacement_angle/4.0);
			servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightFootRoll, -lateral_displacement_angle);

			// This allows the torso upright controller to still actuate over the hip pitch joints
			bool ret_val5 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftHipPitch, -torso_upright_pitch_control_action + last_hip_left_angle_compensation);
			bool ret_val6 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightHipPitch, -torso_upright_pitch_control_action + last_hip_right_angle_compensation);

			servo_updater_.should_be_updated = true;

			if (!trajectory_running)
			{
				state3_finished = true;
				autotare_interpolator1_.reset();
			}
		}
	}
}

void Executor::state4_execution()
{
	// STATE 4: Pivot to the left foot (same as the STATE 1, but without the need to do the first squat).
	// In this state, from home position, the robot pivots to the left in order to stand on the left foot.
	// The movement is divided in two different linear movements.
	// First the pivot to the left, and after that, a small retraction of the right leg to leave more space with the floor.

	bool trajectory_running = false;
	if (0 == state4_phase)
	{
		if (!autotare_interpolator1_.configure_trayectory(0.0, 1.0, autotare_trajectories_time_))
		while (true)
		{
			Serial.println("ERR_BAD_PARAMETER");
			delay(1000);
		}
		state4_phase = 2;
	}
	if (2 == state4_phase)
	{
		if (autotare_interpolator1_.get_execution_flag())
		{
			// TRAJECTORY UNITARY COMPUTATION
			double trajectory_unitary_vector;
			trajectory_running = autotare_interpolator1_.compute_output(trajectory_unitary_vector);

			// LATERAL DISPLACEMENT
			double lateral_displacement_angle = - autotare_angle_displacement_ * trajectory_unitary_vector;

			servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftFootRoll, lateral_displacement_angle);
			servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightHipRoll, -lateral_displacement_angle/4.0);
			servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightFootRoll, -lateral_displacement_angle);

			// This allows the torso upright controller to still actuate over the hip pitch joints
			bool ret_val5 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftHipPitch, -torso_upright_pitch_control_action + last_hip_left_angle_compensation);
			bool ret_val6 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightHipPitch, -torso_upright_pitch_control_action + last_hip_right_angle_compensation);

			servo_updater_.should_be_updated = true;

			if (!trajectory_running)
			{
				state4_phase = 3;
				autotare_interpolator1_.reset();
			}
		}
	}
	if (3 == state4_phase)
	{
		if (autotare_interpolator1_.get_execution_flag())
		{
			// TRAJECTORY UNITARY COMPUTATION
			double trajectory_unitary_vector;
			trajectory_running = autotare_interpolator1_.compute_output(trajectory_unitary_vector);
			
			// LEG LIFT DISPLACEMENT
			double calculated_leg_length = home_position_leg_length_ - autotare_legs_squat_displacement - autotare_leg_lift_displacement * trajectory_unitary_vector;

			//Inverse kinematics
			double forward_angle_rad = 0.0;
			double ankle_target_angle;
			double knee_target_angle;
			double hip_angle_compensation;

			bool ret_val_0 = InverseKinematics::get_leg_joints_angles_from_desired_length_and_orientation(
			calculated_leg_length, forward_angle_rad,
			config_.kinematics.height_knee_ankle, config_.kinematics.height_hip_knee,
			ankle_target_angle, knee_target_angle, hip_angle_compensation);

			// Joint setpoint assignation.
			bool ret_val1 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightFootPitch, ankle_target_angle);
			bool ret_val2 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightKnee, knee_target_angle);
			bool ret_val5 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightHipPitch, -torso_upright_pitch_control_action + hip_angle_compensation);
			bool ret_val6 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftHipPitch, -torso_upright_pitch_control_action + last_hip_left_angle_compensation);
			last_hip_right_angle_compensation = hip_angle_compensation;

			servo_updater_.should_be_updated = true;

			if (!trajectory_running)
			{
				state4_finished = true;
				autotare_interpolator1_.reset();
			}
		}
	}
}

void Executor::state5_execution()
{
	// STATE 5: Waiting.
	// The robot waits a few seconds in order to gather data for the tare of the right foot.

	if (waiting_first_time_)
	{
		waiting_first_time_ = false;
		trajectory_interpolator_.reset();
		waiting_.configure_waiting(waiting_time_);
		if (squats_going_down_) squats_going_down_ = false;
		else squats_going_down_ = true;
	}
	if (waiting_.get_execution_flag())
	{
		if (waiting_.evaluate())
		{
			// Set command to tare right foot.
			command_->commands.force_tare_right = true;
			state5_finished = true;
		}

		// Torso upright posture control is still active during waiting
		// This avoids the outgrow of the integral part of the controller
		bool ret_val5 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftHipPitch, -torso_upright_pitch_control_action + last_hip_left_angle_compensation);
		bool ret_val6 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightHipPitch, -torso_upright_pitch_control_action + last_hip_right_angle_compensation);
	}
}

void Executor::state6_execution()
{
	// STATE 6: Go back to home position.
	// In this state, All movements performed in previous states must be reversed.
	// The movement is divided in three different linear movements.
	// First, the right leg is extended to the same length as the left one, then the pivot movement is reversed,
	// and after that, the squat movement is reversed, back to home position.

	bool trajectory_running = false;
	if (0 == state6_phase)
	{
		if (!autotare_interpolator1_.configure_trayectory(1.0, 0.0, autotare_trajectories_time_))
		while (true)
		{
			Serial.println("ERR_BAD_PARAMETER");
			delay(1000);
		}
		state6_phase = 1;
	}
	if (1 == state6_phase)
	{
		if (autotare_interpolator1_.get_execution_flag())
		{
			// TRAJECTORY UNITARY COMPUTATION
			double trajectory_unitary_vector;
			trajectory_running = autotare_interpolator1_.compute_output(trajectory_unitary_vector);
		
			// LEG LIFT DISPLACEMENT
			double calculated_leg_length = home_position_leg_length_ - autotare_legs_squat_displacement - autotare_leg_lift_displacement * trajectory_unitary_vector;

			//Inverse kinematics
			double forward_angle_rad = 0.0;
			double ankle_target_angle;
			double knee_target_angle;
			double hip_angle_compensation;

			bool ret_val_0 = InverseKinematics::get_leg_joints_angles_from_desired_length_and_orientation(
			calculated_leg_length, forward_angle_rad,
			config_.kinematics.height_knee_ankle, config_.kinematics.height_hip_knee,
			ankle_target_angle, knee_target_angle, hip_angle_compensation);

			// Joint setpoint assignation.
			bool ret_val1 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightFootPitch, ankle_target_angle);
			bool ret_val2 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightKnee, knee_target_angle);
			bool ret_val5 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightHipPitch, -torso_upright_pitch_control_action + hip_angle_compensation);
			bool ret_val6 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftHipPitch, -torso_upright_pitch_control_action + last_hip_left_angle_compensation);
			last_hip_right_angle_compensation = hip_angle_compensation;

			servo_updater_.should_be_updated = true;

			if (!trajectory_running)
			{
				state6_phase = 2;
				autotare_interpolator1_.reset();
			}
		}
	}
	if (2 == state6_phase)
	{
		if (autotare_interpolator1_.get_execution_flag())
		{
			// TRAJECTORY UNITARY COMPUTATION
			double trajectory_unitary_vector;
			trajectory_running = autotare_interpolator1_.compute_output(trajectory_unitary_vector);

			// LATERAL DISPLACEMENT
			double lateral_displacement_angle = - autotare_angle_displacement_ * trajectory_unitary_vector;

			servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftFootRoll, lateral_displacement_angle);
			servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightHipRoll, -lateral_displacement_angle/4.0);
			servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightFootRoll, -lateral_displacement_angle);

			// This allows the torso upright controller to still actuate over the hip pitch joints
			bool ret_val5 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftHipPitch, -torso_upright_pitch_control_action + last_hip_left_angle_compensation);
			bool ret_val6 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightHipPitch, -torso_upright_pitch_control_action + last_hip_right_angle_compensation);

			servo_updater_.should_be_updated = true;

			if (!trajectory_running)
			{
				state6_phase = 3;
				autotare_interpolator1_.reset();
			}
		}
	}
	if (3 == state6_phase)
	{
		if (autotare_interpolator1_.get_execution_flag())
		{
			// TRAJECTORY UNITARY COMPUTATION
			double trajectory_unitary_vector;
			trajectory_running = autotare_interpolator1_.compute_output(trajectory_unitary_vector);
		
			// LEG LIFT DISPLACEMENT
			double calculated_leg_length = home_position_leg_length_ - autotare_legs_squat_displacement * trajectory_unitary_vector;

			//Inverse kinematics
			double forward_angle_rad = 0.0;
			double ankle_target_angle;
			double knee_target_angle;
			double hip_angle_compensation;

			bool ret_val_0 = InverseKinematics::get_leg_joints_angles_from_desired_length_and_orientation(
			calculated_leg_length, forward_angle_rad,
			config_.kinematics.height_knee_ankle, config_.kinematics.height_hip_knee,
			ankle_target_angle, knee_target_angle, hip_angle_compensation);

			// Joint setpoint assignation.
			bool ret_val1 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftFootPitch, ankle_target_angle);
			bool ret_val2 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftKnee, knee_target_angle);
			bool ret_val5 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftHipPitch, -torso_upright_pitch_control_action + hip_angle_compensation);
			bool ret_val3 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightFootPitch, ankle_target_angle);
			bool ret_val4 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightKnee, knee_target_angle);
			bool ret_val6 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightHipPitch, -torso_upright_pitch_control_action + hip_angle_compensation);
			last_hip_left_angle_compensation = hip_angle_compensation;
			last_hip_right_angle_compensation = hip_angle_compensation;

			servo_updater_.should_be_updated = true;
		
			if (!trajectory_running)
			{
				state6_finished = true;
				autotare_interpolator1_.reset();
			}
		}
	}
}

void Executor::state7_execution()
{
	// If sensors have been updated, control can be computed.
	if (gyroscope_accelerometer_manager_.has_been_updated)
	{
		// Squats generation:::
		// Using the trajectory obtained from an interpolator, direct kinematics are applied to form a squat.

		if (squats_first_time_)
		{
			double target_leg_length;
			double current_leg_length;
			if (squats_going_down_)
			{
				target_leg_length = min_desired_leg_length_;
				current_leg_length = max_desired_leg_length_;
			}
			else
			{
				target_leg_length = max_desired_leg_length_;
				current_leg_length = min_desired_leg_length_;
			}
			if (!trajectory_interpolator_.configure_trayectory(target_leg_length, current_leg_length, trajectory_time_ms_))
				while (true)
				{
					Serial.println("ERR_BAD_PARAMETER");
					delay(1000);
				}
			squats_first_time_ = false;
		}

		// Desired leg length computation (linear interpolation)
		interpolator_running_ = trajectory_interpolator_.compute_output(desired_leg_lenght_);

		// Debug prints
		if (command_->commands.squats_debug_on)
		{
			Serial.println("desired_leg_lenght_::: " + (String)desired_leg_lenght_);

			if (command_->commands.squats_debug_off)
			{
				command_->commands.squats_debug_on = false;
				command_->commands.squats_debug_off = false;
			}
		}

		// INVERSE KINEMATICS

		double forward_angle_rad = 0.0;
		double lateral_angle_rad = 0.0;	// Not used
		Vector3d desired_position;
		ForwardKinematics::Geometric::get_position_from_length_and_angles(desired_leg_lenght_, forward_angle_rad, lateral_angle_rad, desired_position);

		Vector2d links_lengths;
		links_lengths << config_.kinematics.height_knee_ankle, config_.kinematics.height_hip_knee;
		double target_angle_rad_1;
		double target_angle_rad_2;
		bool ret_val = InverseKinematics::Geometric::sagittal_two_links_inverse_kinematics(desired_position, links_lengths, target_angle_rad_1, target_angle_rad_2);

		// DIRECT KINEMATICS: OBTAIN TRTANSFORMATION MATRIX to know the orientation of final the effector

		// Denavit-Hartenberg table
		std::vector<Vector4d> DH_table;
		Vector4d DH_row_1;
		DH_row_1 << target_angle_rad_1, 0, config_.kinematics.height_knee_ankle, 0;
		Vector4d DH_row_2;
		DH_row_2 << target_angle_rad_2, 0, config_.kinematics.height_hip_knee, 0;
		DH_table.push_back(DH_row_1);
		DH_table.push_back(DH_row_2);

		Matrix4d TM;

		if (ret_val) ForwardKinematics::DenavitHartenberg::get_overall_TM_from_DH_table(DH_table, TM);

		Matrix2d rotation_matrix = TM.block<2,2>(0,0);

		double angle_to_compensate_hip = 0.0;
		if (ret_val)
		{
			angle_to_compensate_hip = atan2(TM(0,1), TM(0,0));

			// Servo setpoint assignation.
			squats_overlimit_error_propagation_ = false;
			bool ret_val1 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftFootPitch, target_angle_rad_1);
			bool ret_val2 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftKnee, target_angle_rad_2);
			bool ret_val3 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightFootPitch, target_angle_rad_1);
			bool ret_val4 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightKnee, target_angle_rad_2);
			if (!ret_val1 || !ret_val2 || !ret_val3 || !ret_val4)
			{
				// In the case that any of the joints has reached an over-limit angle, the current action is not applied
				servo_updater_.revert_angle_to_joint(Configuration::JointsNames::LeftFootPitch);
				servo_updater_.revert_angle_to_joint(Configuration::JointsNames::LeftKnee);
				servo_updater_.revert_angle_to_joint(Configuration::JointsNames::RightFootPitch);
				servo_updater_.revert_angle_to_joint(Configuration::JointsNames::RightKnee);
				squats_overlimit_error_propagation_ = true;
			}
			else servo_updater_.should_be_updated = true;
		}

		// Servo setpoint assignation.
		bool ret_val5 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftHipPitch, -torso_upright_pitch_control_action + angle_to_compensate_hip);
		bool ret_val6 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightHipPitch, -torso_upright_pitch_control_action + angle_to_compensate_hip);
		if (squats_overlimit_error_propagation_ || !ret_val5 || !ret_val6)
		{
			// In the case that any of the joints has reached an over-limit angle, the current action is not applied
			servo_updater_.revert_angle_to_joint(Configuration::JointsNames::LeftHipPitch);
			servo_updater_.revert_angle_to_joint(Configuration::JointsNames::RightHipPitch);
		}
		else servo_updater_.should_be_updated = true;
	}
}
