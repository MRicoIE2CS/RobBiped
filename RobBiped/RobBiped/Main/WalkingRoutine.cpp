/*
 * WalkingRoutine.h
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

void Executor::state30_execution()
{
	if (state30_first_time)
	{
		state30_first_time = false;

		// Desired hip height
		global_kinematics_.set_desired_hip_height(desired_hip_height_);

		// Reset the CM location to be over the ZMP
		global_kinematics_.init_CoM_location();

		// Set offline reference tracking for Y-axis
		cm_tracking_controller_.set_mode(Control::CMTracking::Mode::OnlineReference, Control::CMTracking::Mode::OfflineReference);
		// And load the trajectories
		cm_tracking_controller_.init();
		// X-axis keeps balance of the CM over the X-axis
		cm_tracking_controller_.set_CM_x_online_reference(0.0);
		// Apply ZMP setpoint for Y-axis ZMP local tracking controllers
		// Each feet will try to center the local ZMP
		left_foot_ZMP_tracking_controller_.set_setpoint_y_mm(0.0);
		right_foot_ZMP_tracking_controller_.set_setpoint_y_mm(0.0);
	}

	if (force_sensors_manager_.has_been_updated)
	{
		// Compute CM tracking control: The output is the ZMP setpoint
		cm_tracking_controller_.compute_ZMP_setpoint();

		// Get current reference point for the CM position
		Vector2d CM_position_reference = cm_tracking_controller_.get_CM_last_reference_location();
		double DSP_CM_setpoint_ = CM_position_reference(1);

		// Compute DSP kinematics
		bool retcode_compute_lateral_DSP_kinematics = global_kinematics_.compute_lateral_DSP_kinematics(DSP_CM_setpoint_);

		// Get roll angle setpoints from DSP kinematics
		double left_roll_angle;
		double right_roll_angle;
		global_kinematics_.get_computed_angles(left_roll_angle, right_roll_angle);

		// Get desired leg lengths computed from DSP kinematics
		double left_leg_length;
		double right_leg_length;
		global_kinematics_.get_computed_prismatic_lengths(left_leg_length, right_leg_length);

		// Get left leg's pitch angle setpoints from DSP kinematics
		double left_ankle_pitch_angle, right_ankle_pitch_angle;
		double left_knee_pitch_angle, right_knee_pitch_angle;
		double left_hip_pitch_angle, right_hip_pitch_angle;
		global_kinematics_.get_joint_angles_for_prismatic_length(left_leg_length, 0.0, left_ankle_pitch_angle, left_knee_pitch_angle, left_hip_pitch_angle);

		// Get right leg's pitch angle setpoints from DSP kinematics
		global_kinematics_.get_joint_angles_for_prismatic_length(right_leg_length, 0.0, right_ankle_pitch_angle, right_knee_pitch_angle, right_hip_pitch_angle);

		// CM tracking control: Get the necessary ZMP setpoint
		Vector2d ZMP_ref_xy = cm_tracking_controller_.get_ZMP_setpoint();

		// Apply new ZMP setpoint for X-axis ZMP tracking controllers
		left_foot_ZMP_tracking_controller_.set_setpoint_x_mm(ZMP_ref_xy(0));
		right_foot_ZMP_tracking_controller_.set_setpoint_x_mm(ZMP_ref_xy(0));

		// Get local ZMP measurements
		Vector2d current_left_ZMP = force_sensors_manager_.get_values_ZMP_LeftFoot();
		Vector2d current_right_ZMP = force_sensors_manager_.get_values_ZMP_RightFoot();

		// Compute ZMP tracking control: The output is the increment no the setpoint angles for ankle joints
		Vector2d left_foot_ZMP_tracking_action = left_foot_ZMP_tracking_controller_.compute(current_left_ZMP(0), current_left_ZMP(1));
		Vector2d right_foot_ZMP_tracking_action = right_foot_ZMP_tracking_controller_.compute(current_right_ZMP(0), current_right_ZMP(1));

		// Apply roll angle setpoints
		bool ret_val1 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftHipRoll, left_roll_angle);
		bool ret_val2 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightHipRoll, -right_roll_angle);
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

		// DEBUG
		//Serial.println("ZMPref_x, ZMPl, lAct, ZMPr, rAct: \t" + (String)ZMP_ref_xy(0) + "\t" + (String)current_left_ZMP(0) + "\t" + (String)left_foot_ZMP_tracking_action(0) + "\t" + (String)current_right_ZMP(0) + "\t" + (String)right_foot_ZMP_tracking_action(0));
		//Serial.println("ZMPref_x, ZMPl, lAct, ZMPr, rAct: \t" + (String)ZMP_ref_xy(1) + "\t" + (String)current_left_ZMP(1) + "\t" + (String)left_foot_ZMP_tracking_action(1) + "\t" + (String)current_right_ZMP(1) + "\t" + (String)right_foot_ZMP_tracking_action(1));

		Vector3d CM_est = global_kinematics_.get_CoM_location();
		Vector2d ZMP_global = force_sensors_manager_.get_global_ZMP();
		Serial.println("CMy_ref, CMy_est, ZMPy_glb, CMx_est, ZMPx_glb: \t" + (String)DSP_CM_setpoint_ + "\t" + (String)CM_est(1) + "\t" + (String)ZMP_global(1) + "\t" + (String)CM_est(0) + "\t" + (String)ZMP_global(0));

		// Set flag to send servo setpoints
		servo_updater_.should_be_updated = true;
	}
}
