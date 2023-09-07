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

#include <functional>

bool lifting_leg_maneouver(double *base_leg_length, double &state30_leg_lifting_target, double *leg_length,
							Control::LinearTrajectoryInterpolator &state30_lifting_leg_interpolator,
							Control::LinearTrajectoryInterpolator &state30_forwarding_leg_interpolator,
							double &state30_leg_lifting_distance_mm, uint32_t &state30_leg_lifting_time_ms, uint32_t &state30_leg_lowering_time_ms,
							bool &state30_lifting_leg, bool &state30_lifting_finished,
							bool &state30_lowering_leg, bool &state30_lowering_leg_finished,
							double *leg_frontal_position, double &state30_leg_frontal_target, double state30_desired_step_length)
{
	double output = 0.0;
	bool maneouver_finished = false;
	if (!state30_lifting_finished && !state30_lifting_leg)
	{
//		Serial.print("state30_lifting_leg\n");
		state30_lifting_leg = true;
		state30_leg_lifting_target = *base_leg_length - state30_leg_lifting_distance_mm;
		state30_lifting_leg_interpolator.configure_trayectory(*base_leg_length, state30_leg_lifting_target, state30_leg_lifting_time_ms);
		state30_lifting_leg = state30_lifting_leg_interpolator.compute_output(output);
// 				Serial.print("Start lifting with\n");
// 				Serial.println("base_left_leg_length:\t" + (String)*base_leg_length);
// 				Serial.println("state30_leg_lifting_target:\t" + (String)state30_leg_lifting_target);
// 				Serial.println("output:\t" + (String)output);
		state30_leg_frontal_target = *leg_frontal_position + state30_desired_step_length;
		state30_forwarding_leg_interpolator.configure_trayectory(*leg_frontal_position, state30_leg_frontal_target, state30_leg_lifting_time_ms+state30_leg_lowering_time_ms);
	}
	else if (!state30_lifting_finished && state30_lifting_leg)
	{
		state30_lifting_leg = state30_lifting_leg_interpolator.compute_output(output);
		if (!state30_lifting_leg) state30_lifting_finished = true;
// 				Serial.print("Compute_output with\n");
// 				Serial.println("output:\t" + (String)output);
// 				Serial.println("state30_lifting_leg:\t" + (String)state30_lifting_leg);
	}
	else if (state30_lifting_finished && !state30_lowering_leg && !state30_lowering_leg_finished)
	{
		Serial.print("state30_lowering_leg\n");
		state30_lowering_leg = true;
		state30_lifting_leg_interpolator.configure_trayectory(state30_leg_lifting_target, *base_leg_length, state30_leg_lowering_time_ms);
		state30_lowering_leg = state30_lifting_leg_interpolator.compute_output(output);
// 				Serial.print("Start lowering with\n");
// 				Serial.println("state30_leg_lifting_target:\t" + (String)state30_leg_lifting_target);
// 				Serial.println("*base_leg_length:\t" + (String)*base_leg_length);
// 				Serial.println("output:\t" + (String)output);
	}
	else if (state30_lowering_leg && !state30_lowering_leg_finished)
	{
		state30_lifting_leg_interpolator.update_target(*base_leg_length);
		state30_lowering_leg = state30_lifting_leg_interpolator.compute_output(output);
// 				Serial.print("Update_target with\n");
// 				Serial.println("*base_leg_length:\t" + (String)*base_leg_length);
// 				Serial.println("output:\t" + (String)output);
		if (!state30_lowering_leg)
		{
//			Serial.print("state30_lowering_leg_finished\n");
			state30_lowering_leg_finished = true;
			maneouver_finished = true;
		}
	}
	else maneouver_finished = true;
	
	double forwarding_output;
	state30_forwarding_leg_interpolator.compute_output(forwarding_output);
	*leg_frontal_position = forwarding_output;
	if (maneouver_finished) *leg_frontal_position = state30_leg_frontal_target;
	
	*leg_length = output;
	return maneouver_finished;
}

void Executor::state30_execution()
{
	// State 30: Walk still routine
	// Command "walk"
	// DSP tracking offline trajectory and swinging leg lifting, without stepping forward
	
	
	if (state30_first_time)
	{
		state30_first_time = false;

		// Desired hip height
		global_kinematics_.set_desired_hip_height(state30_desired_hip_height);
		// Desired step width
		global_kinematics_.set_desired_step_width(state30_desired_step_width);

		// Reset the CM location to be over the ZMP
		global_kinematics_.init_CoM_location();

		// Set offline reference tracking for both axis
		cm_tracking_controller_.set_mode(Control::CMTracking::Mode::OfflineReference, Control::CMTracking::Mode::OfflineReference);
		// And load the trajectories
		cm_tracking_controller_.init();
		// Apply ZMP setpoint for Y-axis ZMP local tracking controllers
		// Each feet will try to center the local ZMP
		left_foot_ZMP_tracking_controller_.set_setpoint_y_mm(0.0);
		right_foot_ZMP_tracking_controller_.set_setpoint_y_mm(0.0);

		// X-axis keeps balance of the CM over the X-axis
		cm_tracking_controller_.set_CM_x_online_reference(0.0);
		cm_tracking_controller_.set_CM_y_online_reference(0.0);

		// Initializations
		GlobalKinematics::WalkingPhase walking_phase = GlobalKinematics::WalkingPhase::DSP_left;
		state30_swing_leg_started = false;
		state30_swing_leg_finished = false;
		state30_lifting_finished = false;
		state30_lifting_leg = false;
		state30_lowering_leg = false;
		state30_lowering_leg_finished = false;
		state30_lifting_leg_controller_reset = false;
		global_kinematics_.lifting_maneuver_performed = false;
		
		debug_millis = millis();
		
//Serial.println("INIT WALKING______________________");
	}

	if (force_sensors_manager_.has_been_updated)
	{
		// COMPUTATIONS COMMON TO ALL PHASES

		// Compute CM tracking control: The output is the ZMP setpoint
		cm_tracking_controller_.compute_ZMP_setpoint();

		// Get current reference point for the CM position
		Vector2d CM_position_reference = cm_tracking_controller_.get_CM_last_reference_location();
		Serial.println("CM_position_reference_X: \t" + (String)CM_position_reference(0));
		
		// Apply parametric curve related to the relation of CM position to hip_center position
		double DSP_CM_setpoint_ = (CM_position_reference(1) - 13.941) / 0.7096;
		
		// WALKING PHASE STATUS
		// Get status of the walking phase.
		// Walking phases are checked every time the force sensors are updated.
		// bool phase_changed = false;
		// GlobalKinematics::WalkingPhase walking_phase = GlobalKinematics::WalkingPhase::DSP_left;
		global_kinematics_.check_walking_phase(DSP_CM_setpoint_);
		bool phase_changed = global_kinematics_.has_there_been_a_phase_change();
		GlobalKinematics::WalkingPhase walking_phase = global_kinematics_.get_current_walking_phase();

// 		// AUTOMATIC ACTIVATION OF ZMP TRACKING CONTROLLERS
// 		if (!left_foot_ZMP_tracking_controller_.is_x_on()) left_foot_ZMP_tracking_controller_.switch_x_on(true);
// 		if (!left_foot_ZMP_tracking_controller_.is_y_on()) left_foot_ZMP_tracking_controller_.switch_y_on(true);
// 		if (!right_foot_ZMP_tracking_controller_.is_x_on()) right_foot_ZMP_tracking_controller_.switch_x_on(true);
// 		if (!right_foot_ZMP_tracking_controller_.is_y_on()) right_foot_ZMP_tracking_controller_.switch_y_on(true);

		// PHASE JUMP ACTIONS
		// If the walking phase has changed in current execution period
// 		if (phase_changed)
// 		{
// 			Serial.print("PHASE CHANGED\t");
// 			// TODO
// 			// TODO: Change to Offline Tracking when going walking in X direction
// 			// TODO
// 			if (GlobalKinematics::WalkingPhase::DSP_left == walking_phase)
// 			{
// 				Serial.print("DSP_left\n");
// 				//cm_tracking_controller_.set_mode(Control::CMTracking::Mode::OnlineReference, Control::CMTracking::Mode::OfflineReference);
// 			}
// 			else if (GlobalKinematics::WalkingPhase::DSP_right == walking_phase)
// 			{
// 				Serial.print("DSP_right\n");
// 				//cm_tracking_controller_.set_mode(Control::CMTracking::Mode::OnlineReference, Control::CMTracking::Mode::OfflineReference);
// 			}
// 			else if (GlobalKinematics::WalkingPhase::SSP_left == walking_phase)
// 			{
// 				Serial.print("SSP_left\n");
// 				//cm_tracking_controller_.set_mode(Control::CMTracking::Mode::OnlineReference, Control::CMTracking::Mode::OfflineReference);
// 				//state30_lifting_finished = false;
// 			}
// 			else if (GlobalKinematics::WalkingPhase::SSP_right == walking_phase)
// 			{
// 				Serial.print("SSP_right\n");
// 				//cm_tracking_controller_.set_mode(Control::CMTracking::Mode::OnlineReference, Control::CMTracking::Mode::OfflineReference);
// 				//state30_lifting_finished = false;
// 			}
// 		}

		// COMPUTATIONS COMMON TO ALL PHASES

		// Compute DSP kinematics
		//bool retcode_compute_lateral_DSP_kinematics = global_kinematics_.compute_lateral_DSP_kinematics(DSP_CM_setpoint_);
		bool retcode_compute_lateral_DSP_kinematics = global_kinematics_.compute_bidimensional_DSP_kinematics(DSP_CM_setpoint_, CM_position_reference(0)*state30_forward_step_length_multiplicity);
Serial.print("CM_position_reference(0)*state30_forward_step_length_multiplicity\n");Serial.println(CM_position_reference(0)*state30_forward_step_length_multiplicity);

		// Get roll angle setpoints from DSP kinematics
		double base_left_roll_angle;
		double base_right_roll_angle;
		global_kinematics_.get_computed_angles(base_left_roll_angle, base_right_roll_angle);

		// Get desired leg lengths computed from DSP kinematics
		double base_left_leg_length;
		double base_right_leg_length;
		global_kinematics_.get_computed_prismatic_lengths(base_left_leg_length, base_right_leg_length);
		double left_leg_length = base_left_leg_length;
		double right_leg_length = base_right_leg_length;
		
		double left_frontal_prismatic_angle, right_frontal_prismatic_angle;
		global_kinematics_.get_computed_frontal_prismatic_angles(left_frontal_prismatic_angle, right_frontal_prismatic_angle);

		// CM tracking control: Get the necessary ZMP setpoint
		Vector2d ZMP_ref_xy = cm_tracking_controller_.get_ZMP_setpoint();

		// PHASE SPECIFIC ACTIONS
		if (GlobalKinematics::WalkingPhase::DSP_left == walking_phase)
		{
//Serial.print("DSP_left\n");
			// Apply new ZMP setpoint for left foot ZMP tracking controllers
			left_foot_ZMP_tracking_controller_.set_setpoint_x_mm(0.0);
			left_foot_ZMP_tracking_controller_.set_setpoint_y_mm(0.0);
//			left_foot_ZMP_tracking_controller_.set_setpoint_x_mm(ZMP_ref_xy(0));
//			left_foot_ZMP_tracking_controller_.set_setpoint_y_mm(ZMP_ref_xy(1));
			// Apply new ZMP setpoint for right foot ZMP tracking controllers
			right_foot_ZMP_tracking_controller_.set_setpoint_x_mm(ZMP_ref_xy(0));
			right_foot_ZMP_tracking_controller_.set_setpoint_y_mm(ZMP_ref_xy(1));

			// LIFTING LEG MANEUVER END
			if (state30_swing_leg_started && state30_swing_leg_finished)
			{
//Serial.print("LIFTING LEG MANEUVER END DSP_left\n");
				state30_swing_leg_started = false;
				state30_swing_leg_finished = false;
				state30_lifting_finished = false;
				state30_lifting_leg = false;
				state30_lowering_leg = false;
				state30_lowering_leg_finished = false;
				state30_lifting_leg_controller_reset = false;
				global_kinematics_.lifting_maneuver_performed = false;
			}
		}
		else if (GlobalKinematics::WalkingPhase::DSP_right == walking_phase)
		{
//Serial.print("DSP_right\n");
			// Apply new ZMP setpoint for left foot ZMP tracking controllers
			left_foot_ZMP_tracking_controller_.set_setpoint_x_mm(ZMP_ref_xy(0));
			left_foot_ZMP_tracking_controller_.set_setpoint_y_mm(ZMP_ref_xy(1));
			// Apply new ZMP setpoint for right foot ZMP tracking controllers
			right_foot_ZMP_tracking_controller_.set_setpoint_x_mm(0.0);
			right_foot_ZMP_tracking_controller_.set_setpoint_y_mm(0.0);
// 			right_foot_ZMP_tracking_controller_.set_setpoint_x_mm(ZMP_ref_xy(0));
// 			right_foot_ZMP_tracking_controller_.set_setpoint_y_mm(ZMP_ref_xy(1));

			// LIFTING LEG MANEUVER END
			if (state30_swing_leg_started && state30_swing_leg_finished)
			{
//Serial.print("LIFTING LEG MANEUVER END DSP_right\n");
				state30_swing_leg_started = false;
				state30_swing_leg_finished = false;
				state30_lifting_finished = false;
				state30_lifting_leg = false;
				state30_lowering_leg = false;
				state30_lowering_leg_finished = false;
				state30_lifting_leg_controller_reset = false;
				global_kinematics_.lifting_maneuver_performed = false;
			}
		}
		else if (GlobalKinematics::WalkingPhase::SSP_left == walking_phase)
		{
//Serial.print("SSP_left\n");
			// Apply new ZMP setpoint for left foot ZMP tracking controllers
			left_foot_ZMP_tracking_controller_.set_setpoint_x_mm(ZMP_ref_xy(0));
			left_foot_ZMP_tracking_controller_.set_setpoint_y_mm(ZMP_ref_xy(1));
			// Apply new ZMP setpoint for right foot ZMP tracking controllers
 			right_foot_ZMP_tracking_controller_.set_setpoint_x_mm(0.0);
 			right_foot_ZMP_tracking_controller_.set_setpoint_y_mm(0.0);
// 			right_foot_ZMP_tracking_controller_.set_setpoint_x_mm(ZMP_ref_xy(0));
// 			right_foot_ZMP_tracking_controller_.set_setpoint_y_mm(ZMP_ref_xy(1));

			// LIFTING LEG MANEUVER
			state30_swing_leg_started = true;
		}
		else if (GlobalKinematics::WalkingPhase::SSP_right == walking_phase)
		{
//Serial.print("SSP_right\n");
			// Apply new ZMP setpoint for left foot ZMP tracking controllers
 			left_foot_ZMP_tracking_controller_.set_setpoint_x_mm(0.0);
 			left_foot_ZMP_tracking_controller_.set_setpoint_y_mm(0.0);
// 			left_foot_ZMP_tracking_controller_.set_setpoint_x_mm(ZMP_ref_xy(0));
// 			left_foot_ZMP_tracking_controller_.set_setpoint_y_mm(ZMP_ref_xy(1));
			// Apply new ZMP setpoint for right foot ZMP tracking controllers
			right_foot_ZMP_tracking_controller_.set_setpoint_x_mm(ZMP_ref_xy(0));
			right_foot_ZMP_tracking_controller_.set_setpoint_y_mm(ZMP_ref_xy(1));

			// LIFTING LEG MANEUVER
			state30_swing_leg_started = true;
		}

		// LIFTING SWING LEG MANEUVER
		if (state30_swing_leg_started && !state30_swing_leg_finished)
		{
			double *base_leg_legth, *leg_length;
			double *pointer_leg_frontal_position;
			if (GlobalKinematics::WalkingPhase::SSP_right == walking_phase || GlobalKinematics::WalkingPhase::DSP_left == walking_phase)
			{
				base_leg_legth = &base_left_leg_length;
				leg_length = &left_leg_length;
				global_kinematics_.get_modfiable_left_frontal_position(pointer_leg_frontal_position);
Serial.print("LEFTpointer_leg_frontal_position\n");Serial.println(*pointer_leg_frontal_position);
				// Reset lifting leg's ankle controller when it is lifted
				if (!state30_swing_leg_finished && state30_lifting_finished && !state30_lifting_leg_controller_reset)
				{
					state30_lifting_leg_controller_reset = true;
					left_foot_ZMP_tracking_controller_.reset_value_x();
					left_foot_ZMP_tracking_controller_.reset_value_y();
				}
			}
			else if (GlobalKinematics::WalkingPhase::SSP_left == walking_phase || GlobalKinematics::WalkingPhase::DSP_right == walking_phase)
			{
				base_leg_legth = &base_right_leg_length;
				leg_length = &right_leg_length;
				global_kinematics_.get_modfiable_right_frontal_position(pointer_leg_frontal_position);
Serial.print("RIGHTpointer_leg_frontal_position\n");Serial.println(*pointer_leg_frontal_position);
				// Reset lifting leg's ankle controller when it is lifted
				if (!state30_swing_leg_finished && state30_lifting_finished && !state30_lifting_leg_controller_reset)
				{
					state30_lifting_leg_controller_reset = true;
					right_foot_ZMP_tracking_controller_.reset_value_x();
					right_foot_ZMP_tracking_controller_.reset_value_y();
				}
			}
			
			state30_swing_leg_finished = lifting_leg_maneouver(base_leg_legth, state30_leg_lifting_target, leg_length,
															state30_lifting_leg_interpolator, state30_forwarding_leg_interpolator,
															state30_leg_lifting_distance_mm, state30_leg_lifting_time_ms, state30_leg_lowering_time_ms,
															state30_lifting_leg, state30_lifting_finished,
															state30_lowering_leg, state30_lowering_leg_finished,
															pointer_leg_frontal_position, state30_leg_frontal_target, 2.0 * state30_desired_step_length * state30_forward_step_length_multiplicity);
			// Maneuver finished
			if (state30_swing_leg_finished) global_kinematics_.lifting_maneuver_performed = true;
		}

		// COMPUTATIONS COMMON TO ALL PHASES

		// Get left leg's pitch angle setpoints from DSP kinematics
		double left_ankle_pitch_angle, right_ankle_pitch_angle;
		double left_knee_pitch_angle, right_knee_pitch_angle;
		double left_hip_pitch_angle, right_hip_pitch_angle;
		// TODO: Apply frontal inclination
		global_kinematics_.get_joint_angles_for_prismatic_length(left_leg_length, left_frontal_prismatic_angle, left_ankle_pitch_angle, left_knee_pitch_angle, left_hip_pitch_angle);

		// Get right leg's pitch angle setpoints from DSP kinematics
		global_kinematics_.get_joint_angles_for_prismatic_length(right_leg_length, right_frontal_prismatic_angle, right_ankle_pitch_angle, right_knee_pitch_angle, right_hip_pitch_angle);

		// Get local ZMP measurements
		Vector2d current_left_ZMP = force_sensors_manager_.get_values_ZMP_LeftFoot();
		Vector2d current_right_ZMP = force_sensors_manager_.get_values_ZMP_RightFoot();

		// Compute ZMP tracking control: The output is the increment no the setpoint angles for ankle joints
		Vector2d left_foot_ZMP_tracking_action = left_foot_ZMP_tracking_controller_.compute(current_left_ZMP(0), current_left_ZMP(1));
		Vector2d right_foot_ZMP_tracking_action = right_foot_ZMP_tracking_controller_.compute(current_right_ZMP(0), current_right_ZMP(1));

		// SETPOINT ASSIGNATIONS

		// Apply roll angle setpoints
		double compensated_left_hip_roll_angle = global_kinematics_.compensate_hip_roll_angle(base_left_roll_angle, false);
		double compensated_right_hip_roll_angle = global_kinematics_.compensate_hip_roll_angle(-base_right_roll_angle, true);
		servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftHipRoll, compensated_left_hip_roll_angle);
		servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightHipRoll, compensated_right_hip_roll_angle);
		servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftFootRoll, -base_left_roll_angle + left_foot_ZMP_tracking_action(1));
		servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightFootRoll, base_right_roll_angle + right_foot_ZMP_tracking_action(1));

		// Apply left leg's pitch angle setpoints
		servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftFootPitch, left_ankle_pitch_angle + left_foot_ZMP_tracking_action(0));
		servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftKnee, left_knee_pitch_angle);
		servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftHipPitch, left_hip_pitch_angle + torso_upright_pitch_control_action);

		// Apply right leg's pitch angle setpoints
		servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightFootPitch, right_ankle_pitch_angle + right_foot_ZMP_tracking_action(0));
		servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightKnee, right_knee_pitch_angle);
		servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightHipPitch, right_hip_pitch_angle + torso_upright_pitch_control_action);
		

		

		// DEBUG
		//Serial.println("ZMPref_x, ZMPl, lAct, ZMPr, rAct: \t" + (String)ZMP_ref_xy(0) + "\t" + (String)current_left_ZMP(0) + "\t" + (String)left_foot_ZMP_tracking_action(0) + "\t" + (String)current_right_ZMP(0) + "\t" + (String)right_foot_ZMP_tracking_action(0));
		//Serial.println("ZMPref_x, ZMPl, lAct, ZMPr, rAct: \t" + (String)ZMP_ref_xy(1) + "\t" + (String)current_left_ZMP(1) + "\t" + (String)left_foot_ZMP_tracking_action(1) + "\t" + (String)current_right_ZMP(1) + "\t" + (String)right_foot_ZMP_tracking_action(1));

// 		Vector3d CM_est = global_kinematics_.get_CoM_location();
// 		Vector2d ZMP_global = force_sensors_manager_.get_global_ZMP();
//		Serial.println("CMy_ref, CMy_est, ZMPy_glb, CMx_est, ZMPx_glb: \t" + (String)DSP_CM_setpoint_ + "\t" + (String)CM_est(1) + "\t" + (String)ZMP_global(1) + "\t" + (String)CM_est(0) + "\t" + (String)ZMP_global(0));

		if (millis() - last_print_millis > 50)
		{
			Vector3d aCM = global_kinematics_.get_CoM_acceleration();
			Vector3d vCM = global_kinematics_.get_CoM_velocity();
			Vector3d CM = global_kinematics_.get_CoM_location();
			Vector2d ZMP = force_sensors_manager_.get_global_ZMP();
			String controllers_on = "";
			controllers_on += (left_foot_ZMP_tracking_controller_.is_x_on()) ? "1" : "0";
			controllers_on += (left_foot_ZMP_tracking_controller_.is_y_on()) ? "1" : "0";
			controllers_on += (right_foot_ZMP_tracking_controller_.is_x_on()) ? "1" : "0";
			controllers_on += (right_foot_ZMP_tracking_controller_.is_y_on()) ? "1" : "0";
			String walking_phase_s = "";
			walking_phase_s += (GlobalKinematics::WalkingPhase::DSP_left == walking_phase) ? "1" : "0";
			walking_phase_s += (GlobalKinematics::WalkingPhase::DSP_right == walking_phase) ? "1" : "0";
			walking_phase_s += (GlobalKinematics::WalkingPhase::SSP_left == walking_phase) ? "1" : "0";
			walking_phase_s += (GlobalKinematics::WalkingPhase::SSP_right == walking_phase) ? "1" : "0";
			Serial.println("Yaxis-> timestamp, CMref, CM, ZMP, {lx_on ly_on rx_on ry_on}, {lx_on ly_on rx_on ry_on}: \t" + (String)(millis() - debug_millis) + "\t" + (String)CM_position_reference(1) + "\t" + (String)some_exp_filter_3_.filter(CM(1)) + "\t" + (String)ZMP(1) + "\t" + controllers_on + "\t" + walking_phase_s);
			
			last_print_millis = millis();
		}

		// Set flag to send servo setpoints
		servo_updater_.should_be_updated = true;
	}
}
