/*
 * TestsRoutines.h
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

 void Executor::state1_execution()
 {
	 // STATE 1: Manual DSP movement for hip height and Y position
	 // Command "kinematics"

	 if (state1_first_time)
	 {
		 state1_first_time = false;
		 
		 global_kinematics_.set_desired_hip_height(state1_desired_hip_height);

		 global_kinematics_.set_desired_step_width(state1_desired_step_width);
	 }

	 if (force_sensors_manager_.has_been_updated)
	 {
		 // Potentiometer value sets the CM setpoint in Double Support Phase, along the Y-axis
		 double potentiometer_value = some_exp_filter_.filter(user_input_.get_analog_value(UserInput::AnalogInputList::potentiometer1) / 4095.0);
		 // Desired hip height
		 double desired_hip_height = config_.kinematics.limit_down_hip_height + (config_.kinematics.limit_up_hip_height - config_.kinematics.limit_down_hip_height) * potentiometer_value;
		 Serial.println("desired_hip_height: \t" + (String)desired_hip_height);

		 global_kinematics_.set_desired_hip_height(desired_hip_height);
		 
		 // Potentiometer value sets the CM setpoint in Double Support Phase, along the Y-axis
		 double potentiometer_value2 = some_exp_filter_2_.filter(user_input_.get_analog_value(UserInput::AnalogInputList::potentiometer2) / 4095.0);
		 
		 // Desired CM position
		 double DSP_CM_setpoint_ = right_foot_center_ - state1_distance_addition + (state20_desired_step_width + 2.0*state1_distance_addition) * potentiometer_value2;
		 Serial.println("DSP_CM_setpoint_: \t" + (String)DSP_CM_setpoint_);
		 
		 bool retcode_compute_lateral_DSP_kinematics = global_kinematics_.compute_lateral_DSP_kinematics(DSP_CM_setpoint_);
		 
		 double left_hip_roll, right_foot_roll;
		 global_kinematics_.get_computed_angles(left_hip_roll, right_foot_roll);

		 bool ret_val1 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftHipRoll, left_hip_roll);
		 bool ret_val2 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightHipRoll, -right_foot_roll);
		 bool ret_val3 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftFootRoll, -left_hip_roll);
		 bool ret_val4 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightFootRoll, right_foot_roll);

		 double left_prismatic_length, right_prismatic_length;	// Both legs will have same length
		 global_kinematics_.get_computed_prismatic_lengths(left_prismatic_length, right_prismatic_length);
		 double ankle_pitch_angle;
		 double knee_pitch_angle;
		 double hip_pitch_angle;
		 bool ret_val5 = global_kinematics_.get_joint_angles_for_prismatic_length(left_prismatic_length, 0.0, ankle_pitch_angle, knee_pitch_angle, hip_pitch_angle);

		 ret_val1 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftFootPitch, ankle_pitch_angle);
		 ret_val2 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftKnee, knee_pitch_angle);
		 ret_val3 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftHipPitch, hip_pitch_angle + torso_upright_pitch_control_action);

		 ret_val1 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightFootPitch, ankle_pitch_angle);
		 ret_val2 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightKnee, knee_pitch_angle);
		 ret_val3 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightHipPitch, hip_pitch_angle + torso_upright_pitch_control_action);

		 if (millis() - last_print_millis > 50)
		 {
			 Vector3d aCM = global_kinematics_.get_CoM_acceleration();
			 Vector3d vCM = global_kinematics_.get_CoM_velocity();
			 Vector3d CM = global_kinematics_.get_CoM_location();
			 Vector2d ZMP = force_sensors_manager_.get_global_ZMP();
			 Serial.println("Yaxis-> timestamp, setpoint, CM, vCM, aCM, ZMP: \t" + (String)(millis() - debug_millis) + "\t" + (String)DSP_CM_setpoint_ + "\t" + (String)CM(1) + "\t" + (String)vCM(1) + "\t" + (String)aCM(1) + "\t" + (String)ZMP(1));
			 last_print_millis = millis();
		 }
	 }
 }

 void Executor::state2_execution()
 {
	 // STATE 2: GO UP AND DOWN ROUTINE
	 // Command "get.up"

	 if (state2_first_time)
	 {
		 state2_first_time = false;

		 double desired_step_width = 100.0;
		 global_kinematics_.set_desired_step_width(desired_step_width);
	 }

	 if (force_sensors_manager_.has_been_updated)
	 {
		 // Potentiometer value sets the CM setpoint in Double Support Phase, along the Y-axis
		 double potentiometer_value = some_exp_filter_.filter(user_input_.get_analog_value(UserInput::AnalogInputList::potentiometer1) / 4095.0);
		 // Desired hip height
		 double desired_hip_height = config_.kinematics.limit_down_hip_height + (config_.kinematics.limit_up_hip_height - config_.kinematics.limit_down_hip_height) * potentiometer_value;
		 Serial.println("desired_hip_height: \t" + (String)desired_hip_height);

		 global_kinematics_.set_desired_hip_height(desired_hip_height);
		 bool retcode_compute_lateral_DSP_kinematics = global_kinematics_.compute_lateral_DSP_kinematics(50.0);
 
		 double left_hip_roll, right_foot_roll;
		 global_kinematics_.get_computed_angles(left_hip_roll, right_foot_roll);

		 bool ret_val1 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftHipRoll, left_hip_roll);
		 bool ret_val2 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightHipRoll, -right_foot_roll);
		 bool ret_val3 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftFootRoll, -left_hip_roll);
		 bool ret_val4 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightFootRoll, right_foot_roll);

		 double left_prismatic_length, right_prismatic_length;	// Both legs will have same length
		 global_kinematics_.get_computed_prismatic_lengths(left_prismatic_length, right_prismatic_length);
		 double ankle_pitch_angle;
		 double knee_pitch_angle;
		 double hip_pitch_angle;
		 bool ret_val5 = global_kinematics_.get_joint_angles_for_prismatic_length(left_prismatic_length, 0.0, ankle_pitch_angle, knee_pitch_angle, hip_pitch_angle);

		 ret_val1 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftFootPitch, ankle_pitch_angle);
		 ret_val2 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftKnee, knee_pitch_angle);
		 ret_val3 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftHipPitch, hip_pitch_angle + torso_upright_pitch_control_action);

		 ret_val1 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightFootPitch, ankle_pitch_angle);
		 ret_val2 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightKnee, knee_pitch_angle);
		 ret_val3 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightHipPitch, hip_pitch_angle + torso_upright_pitch_control_action);
	 }
 }
 
 void Executor::state10_execution()
 {
	 // TEST: X BALANCE IN DSP
	 // Command "xbalance"

	 if (state10_first_time)
	 {
		 state10_first_time = false;
		 global_kinematics_.set_desired_hip_height(desired_hip_height_);
		 global_kinematics_.init_CoM_location();
		 // Set offline reference tracking for Y-axis
		 cm_tracking_controller_.set_mode(Control::CMTracking::Mode::OnlineReference, Control::CMTracking::Mode::OnlineReference);
		 // And load the trajectories
		 cm_tracking_controller_.init();
		 // X-axis keeps balance of the CM over the X-axis
		 cm_tracking_controller_.set_CM_x_online_reference(0.0);

		 debug_millis = millis();
	 }

	 if (force_sensors_manager_.has_been_updated)
	 {
		 // Potentiometer value sets the CM setpoint in Double Support Phase, along the Y-axis
		 double potentiometer_value1 = some_exp_filter_.filter(user_input_.get_analog_value(UserInput::AnalogInputList::potentiometer1) / 4095.0);
		 // Desired CM position
		 double DSP_CM_setpoint_ = right_foot_center_ + 20 + (desired_step_width_ - 40) * potentiometer_value1;

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
		 //left_leg_length = left_leg_length - config_.kinematics.height_hip - config_.kinematics.height_ankle;
		 double ankle_pitch_angle;
		 double knee_pitch_angle;
		 double hip_pitch_angle;
		 global_kinematics_.get_joint_angles_for_prismatic_length(left_leg_length, 0.0, ankle_pitch_angle, knee_pitch_angle, hip_pitch_angle);

		 // Get right leg's pitch angle setpoints from DSP kinematics
		 //right_leg_length = right_leg_length - config_.kinematics.height_hip - config_.kinematics.height_ankle;
		 global_kinematics_.get_joint_angles_for_prismatic_length(right_leg_length, 0.0, ankle_pitch_angle, knee_pitch_angle, hip_pitch_angle);

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
		 bool ret_val1 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftHipRoll, left_roll_angle);
		 bool ret_val2 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightHipRoll, -right_roll_angle);
		 bool ret_val3 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftFootRoll, -left_roll_angle + left_foot_ZMP_tracking_action(1));
		 bool ret_val4 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightFootRoll, right_roll_angle + right_foot_ZMP_tracking_action(1));

		 // Apply left leg's pitch angle setpoints
		 ret_val1 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftFootPitch, ankle_pitch_angle + left_foot_ZMP_tracking_action(0));
		 ret_val2 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftKnee, knee_pitch_angle);
		 ret_val3 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::LeftHipPitch, hip_pitch_angle + torso_upright_pitch_control_action);

		 // Apply right leg's pitch angle setpoints
		 ret_val1 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightFootPitch, ankle_pitch_angle + right_foot_ZMP_tracking_action(0));
		 ret_val2 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightKnee, knee_pitch_angle);
		 ret_val3 = servo_updater_.set_angle_to_joint(Configuration::JointsNames::RightHipPitch, hip_pitch_angle + torso_upright_pitch_control_action);

		 // Prints for CM estimation testing in Y and X axis
		 // Reset time for debug
		 if (command_->commands.reset_time)
		 {
			 command_->commands.reset_time = false;
			 debug_millis = millis();
		 }
		 Vector3d aCM = global_kinematics_.get_CoM_acceleration();
		 Vector3d vCM = global_kinematics_.get_CoM_velocity();
		 Vector3d CM = global_kinematics_.get_CoM_location();
		 Vector2d ZMP = force_sensors_manager_.get_global_ZMP();
		 Serial.println("Xaxis-> timestamp, CM, vCM, aCM, ZMP: \t" + (String)(millis() - debug_millis) + "\t" + (String)CM(0) + "\t" + (String)vCM(0) + "\t" + (String)aCM(0) + "\t" + (String)ZMP(0));

		 // Set flag to send servo setpoints
		 servo_updater_.should_be_updated = true;
	 }
 }

void Executor::state20_execution()
{
	// TEST: DSP MOVEMENT FOLLOWING SIN SIGNAL
	// Command "DSPsin"
	 
	if (state20_first_time)
	{
		state20_first_time = false;

		global_kinematics_.set_desired_hip_height(state20_desired_hip_height);
		global_kinematics_.set_desired_step_width(state20_desired_step_width);
		 
		global_kinematics_.init_CoM_location();
		// Set offline reference tracking for Y-axis
		cm_tracking_controller_.set_mode(Control::CMTracking::Mode::OnlineReference, Control::CMTracking::Mode::OnlineReference);
		// And load the trajectories
		cm_tracking_controller_.init();
		// X-axis keeps balance of the CM over the X-axis
		cm_tracking_controller_.set_CM_x_online_reference(0.0);

		// Sin signal configuration
		state20_sin_signal.configure_signal(SignalGenerator::SignalType::sine, state20_sin_period, 1.0, 0.5, 0.0);
		state20_sin_signal.init();
		debug_millis = millis();
	}

	if (force_sensors_manager_.has_been_updated)
	{
		// Sinusoidal signal value sets the CM setpoint in Double Support Phase, along the Y-axis
		double sin_value = state20_sin_signal.generate_trajectory();

		// Desired CM position
		double DSP_CM_setpoint_ = right_foot_center_ - state20_distance_addition + (state20_desired_step_width + 2.0*state20_distance_addition) * sin_value;
		//Serial.println("DSP_CM_setpoint_: \t" + (String)DSP_CM_setpoint_);

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
		//left_leg_length = left_leg_length - config_.kinematics.height_hip - config_.kinematics.height_ankle;
		double left_ankle_pitch_angle, right_ankle_pitch_angle;
		double left_knee_pitch_angle, right_knee_pitch_angle;
		double left_hip_pitch_angle, right_hip_pitch_angle;
		global_kinematics_.get_joint_angles_for_prismatic_length(left_leg_length, 0.0, left_ankle_pitch_angle, left_knee_pitch_angle, left_hip_pitch_angle);

		// Get right leg's pitch angle setpoints from DSP kinematics
		//right_leg_length = right_leg_length - config_.kinematics.height_hip - config_.kinematics.height_ankle;
		global_kinematics_.get_joint_angles_for_prismatic_length(right_leg_length, 0.0, right_ankle_pitch_angle, right_knee_pitch_angle, right_hip_pitch_angle);

		// Compute CM tracking control: The output is the ZMP setpoint
		Vector2d ZMP_ref_xy = cm_tracking_controller_.compute_ZMP_setpoint();
		// ZMP centering in Y direction
		ZMP_ref_xy(1) = 0.0;
		 
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
// 		double compensated_left_roll_angle = left_roll_angle;
// 		double compensated_right_roll_angle = -right_roll_angle;
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

		// SIGNAL PRINTS
		//Serial.println("ZMPref_x, ZMPl, lAct, ZMPr, rAct: \t" + (String)ZMP_ref_xy(0) + "\t" + (String)current_left_ZMP(0) + "\t" + (String)left_foot_ZMP_tracking_action(0) + "\t" + (String)current_right_ZMP(0) + "\t" + (String)right_foot_ZMP_tracking_action(0));
		
		// Prints for CM estimation testing in Y and X axis
		// Reset time for debug
		if (command_->commands.reset_time)
		{
			command_->commands.reset_time = false;
			debug_millis = millis();
		}
		if (millis() - last_print_millis > 50)
		{
			Vector3d aCM = global_kinematics_.get_CoM_acceleration();
			Vector3d vCM = global_kinematics_.get_CoM_velocity();
			Vector3d CM = global_kinematics_.get_CoM_location();
			Vector2d ZMP = force_sensors_manager_.get_global_ZMP();
			Serial.println("Yaxis-> timestamp, setpoint, CM, vCM, aCM, ZMP: \t" + (String)(millis() - debug_millis) + "\t" + (String)DSP_CM_setpoint_ + "\t" + (String)CM(1) + "\t" + (String)vCM(1) + "\t" + (String)aCM(1) + "\t" + (String)ZMP(1));
			last_print_millis = millis();
		}
		//Serial.println("Xaxis-> timestamp, setpoint, CM, vCM, aCM, ZMP: \t" + (String)(millis() - debug_millis) + "\t" + (String)DSP_CM_setpoint_ + "\t" + (String)CM(0) + "\t" + (String)vCM(0) + "\t" + (String)aCM(0) + "\t" + (String)ZMP(0));

	}
}

void Executor::state40_execution()
{
	// TEST: ZMP TRACKING
	// Command "zmptr_test"
	// The robot stands upwards, and the ZMP tracking controllers are active on each foot as long it is touching ground.
	// With the buttons, each leg is lifted, for the other to be the one touching ground.
	// The robot must be kept balance by hand of the user. Otherwise the robot will fall.
	// Then, with both potentiometers, the reference setpoints are set for the ZMP over the X and Y axis.
	
	if (state40_first_time)
	{
		state40_first_time = false;

		global_kinematics_.set_desired_hip_height(state40_desired_hip_height);
		global_kinematics_.set_desired_step_width(state40_desired_step_width);
		
		global_kinematics_.init_CoM_location();

		debug_millis = millis();
		
		// Compute DSP kinematics
		bool retcode_compute_lateral_DSP_kinematics = global_kinematics_.compute_lateral_DSP_kinematics(state40_desired_step_width / 2.0);
	}

	if (force_sensors_manager_.has_been_updated)
	{
		// AUTOMATIC DE/ACTIVATION OF ZMP TRACKING CONTROLLERS
		if (!force_sensors_manager_.is_left_foot_touching_ground()) {
			if (left_foot_ZMP_tracking_controller_.is_x_on()) left_foot_ZMP_tracking_controller_.switch_x_off(true);
			if (left_foot_ZMP_tracking_controller_.is_y_on()) left_foot_ZMP_tracking_controller_.switch_y_off(true); }
		else {
			if (!left_foot_ZMP_tracking_controller_.is_x_on()) left_foot_ZMP_tracking_controller_.switch_x_on(true);
			if (!left_foot_ZMP_tracking_controller_.is_y_on()) left_foot_ZMP_tracking_controller_.switch_y_on(true); }
		if (!force_sensors_manager_.is_right_foot_touching_ground()) {
			if (right_foot_ZMP_tracking_controller_.is_x_on()) right_foot_ZMP_tracking_controller_.switch_x_off(true);
			if (right_foot_ZMP_tracking_controller_.is_y_on()) right_foot_ZMP_tracking_controller_.switch_y_off(true); }
		else {
			if (!right_foot_ZMP_tracking_controller_.is_x_on()) right_foot_ZMP_tracking_controller_.switch_x_on(true);
			if (!right_foot_ZMP_tracking_controller_.is_y_on()) right_foot_ZMP_tracking_controller_.switch_y_on(true); }

		// Get roll angle setpoints from DSP kinematics
		double left_roll_angle;
		double right_roll_angle;
		global_kinematics_.get_computed_angles(left_roll_angle, right_roll_angle);

		// Get desired leg lengths computed from DSP kinematics
		double left_leg_length;
		double right_leg_length;
		global_kinematics_.get_computed_prismatic_lengths(left_leg_length, right_leg_length);
		
		{
		uint32_t current_millis = millis();
		if (user_input_.get_digital_value(UserInput::DigitalInputList::forward_button) && (current_millis - state40_last_millis_for_buttons > state40_delay_for_buttons))
		{
			state40_left_leg_lifted ? state40_left_leg_lifted = false : state40_left_leg_lifted = true;
			state40_last_millis_for_buttons = current_millis;
		}
		if (user_input_.get_digital_value(UserInput::DigitalInputList::back_button) && (current_millis - state40_last_millis_for_buttons > state40_delay_for_buttons))
		{
			state40_right_leg_lifted ? state40_right_leg_lifted = false : state40_right_leg_lifted = true;
			state40_last_millis_for_buttons = current_millis;
		}
		}
		
		
		if (state40_left_leg_lifted)
		{
			left_leg_length = left_leg_length - state40_distance_to_lift;
		}
		if (state40_right_leg_lifted)
		{
			right_leg_length = right_leg_length - state40_distance_to_lift;
		}

		// Get left leg's pitch angle setpoints from DSP kinematics
		//left_leg_length = left_leg_length - config_.kinematics.height_hip - config_.kinematics.height_ankle;
		double left_ankle_pitch_angle, right_ankle_pitch_angle;
		double left_knee_pitch_angle, right_knee_pitch_angle;
		double left_hip_pitch_angle, right_hip_pitch_angle;
		global_kinematics_.get_joint_angles_for_prismatic_length(left_leg_length, 0.0, left_ankle_pitch_angle, left_knee_pitch_angle, left_hip_pitch_angle);

		// Get right leg's pitch angle setpoints from DSP kinematics
		//right_leg_length = right_leg_length - config_.kinematics.height_hip - config_.kinematics.height_ankle;
		global_kinematics_.get_joint_angles_for_prismatic_length(right_leg_length, 0.0, right_ankle_pitch_angle, right_knee_pitch_angle, right_hip_pitch_angle);
		
		// Setting a ZMP reference to track from the potentiometer2 signal.
		// Potentiometer1 value sets the ZMP setpoint, along the X-axis
		double potentiometer_value1 = some_exp_filter_.filter(user_input_.get_analog_value(UserInput::AnalogInputList::potentiometer1) / 4095.0);
		// Potentiometer2 value sets the ZMP setpoint, along the Y-axis
		double potentiometer_value2 = some_exp_filter_2_.filter(user_input_.get_analog_value(UserInput::AnalogInputList::potentiometer2) / 4095.0);
		// Desired CM position
		Vector2d ZMP_ref_xy;
		ZMP_ref_xy(0) = -config_.force_sensors.location_mm.frontBack_separation/2.0 + config_.force_sensors.location_mm.frontBack_separation * potentiometer_value1;
		ZMP_ref_xy(1) = -config_.force_sensors.location_mm.leftRight_separation/2.0 + config_.force_sensors.location_mm.leftRight_separation * potentiometer_value2;

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

		// SIGNAL PRINTS
		
		// Prints for CM estimation testing in Y and X axis
		// Reset time for debug
		if (command_->commands.reset_time)
		{
			command_->commands.reset_time = false;
			debug_millis = millis();
		}
		//Vector3d aCM = global_kinematics_.get_CoM_acceleration();
		//Vector3d vCM = global_kinematics_.get_CoM_velocity();
		//Vector3d CM = global_kinematics_.get_CoM_location();
		Vector2d ZMP = force_sensors_manager_.get_global_ZMP();
		Serial.println("timestamp, setpointX, ZMPX, setpointYr, setpointYl, ZMPY: \t" + 
						(String)(millis() - debug_millis) + "\t" + (String)ZMP_ref_xy(0) + "\t" + (String)ZMP(0) + "\t" +
						(String)ZMP_ref_xy(1) + "\t" + (String)(ZMP_ref_xy(1) + state40_desired_step_width) + "\t" + 
						(String)ZMP(1) + "\t" + 
						(String)right_foot_ZMP_tracking_action(1) + "\t" + (String)left_foot_ZMP_tracking_action(1));
		
	}
}

void Executor::state50_execution()
{
	// TEST: SSP BALANCE
	// Command "SSPbalance"
	// The robot stands upwards, and the ZMP tracking controllers are active on each foot as long it is touching ground.
	// With the buttons, each leg is lifted, for the other to be the one touching ground.
	// The robot is intended to keep balance by itself due to CM stabilizer and ZMP tracking controllers.
	// With both potentiometers, the DSP hip center and the hip height setpoints are set.
	
	if (state50_first_time)
	{
		state50_first_time = false;

		global_kinematics_.set_desired_step_width(state50_desired_step_width);
		
		global_kinematics_.init_CoM_location();

		// Set offline reference tracking for Y-axis
		cm_tracking_controller_.set_mode(Control::CMTracking::Mode::OnlineReference, Control::CMTracking::Mode::OnlineReference);
		// And load the trajectories
		cm_tracking_controller_.init();
		// X-axis keeps balance of the CM over the X-axis
		cm_tracking_controller_.set_CM_x_online_reference(0.0);

		debug_millis = millis();
		
	}

	if (force_sensors_manager_.has_been_updated)
	{
		// AUTOMATIC DE/ACTIVATION OF ZMP TRACKING CONTROLLERS
		if (!force_sensors_manager_.is_left_foot_touching_ground())
		{
			if (left_foot_ZMP_tracking_controller_.is_x_on()) left_foot_ZMP_tracking_controller_.switch_x_off(true);
			if (left_foot_ZMP_tracking_controller_.is_y_on()) left_foot_ZMP_tracking_controller_.switch_y_off(true);
		}
		else
		{
			if (!left_foot_ZMP_tracking_controller_.is_x_on()) left_foot_ZMP_tracking_controller_.switch_x_on(true);
			if (!left_foot_ZMP_tracking_controller_.is_y_on()) left_foot_ZMP_tracking_controller_.switch_y_on(true);
		}
		if (!force_sensors_manager_.is_right_foot_touching_ground())
		{
			if (right_foot_ZMP_tracking_controller_.is_x_on()) right_foot_ZMP_tracking_controller_.switch_x_off(true);
			if (right_foot_ZMP_tracking_controller_.is_y_on()) right_foot_ZMP_tracking_controller_.switch_y_off(true);
		}
		else
		{
			if (!right_foot_ZMP_tracking_controller_.is_x_on()) right_foot_ZMP_tracking_controller_.switch_x_on(true);
			if (!right_foot_ZMP_tracking_controller_.is_y_on()) right_foot_ZMP_tracking_controller_.switch_y_on(true);
		}

		// Potentiometer value sets the CM setpoint in Double Support Phase, along the Y-axis
		double potentiometer_value = some_exp_filter_.filter(user_input_.get_analog_value(UserInput::AnalogInputList::potentiometer1) / 4095.0);
		// Desired hip height
		double desired_hip_height = config_.kinematics.limit_down_hip_height + (config_.kinematics.limit_up_hip_height - config_.kinematics.limit_down_hip_height) * potentiometer_value;
		//Serial.println("desired_hip_height: \t" + (String)desired_hip_height);

		global_kinematics_.set_desired_hip_height(desired_hip_height);

		// Potentiometer value sets the CM setpoint in Double Support Phase, along the Y-axis
		double potentiometer_value2 = some_exp_filter_2_.filter(user_input_.get_analog_value(UserInput::AnalogInputList::potentiometer2) / 4095.0);

		// Desired CM position
		double DSP_CM_setpoint_ = right_foot_center_ - state50_distance_addition + (state50_desired_step_width + 2.0*state50_distance_addition) * potentiometer_value2;

		bool retcode_compute_lateral_DSP_kinematics = global_kinematics_.compute_lateral_DSP_kinematics(DSP_CM_setpoint_);

		// Get roll angle setpoints from DSP kinematics
		double left_roll_angle;
		double right_roll_angle;
		global_kinematics_.get_computed_angles(left_roll_angle, right_roll_angle);

		// Get desired leg lengths computed from DSP kinematics
		double left_leg_length;
		double right_leg_length;
		global_kinematics_.get_computed_prismatic_lengths(left_leg_length, right_leg_length);
		
		{
			uint32_t current_millis = millis();
			if (user_input_.get_digital_value(UserInput::DigitalInputList::forward_button) && (current_millis - state50_last_millis_for_buttons > state50_delay_for_buttons))
			{
				state50_left_leg_lifted ? state50_left_leg_lifted = false : state50_left_leg_lifted = true;
				state50_last_millis_for_buttons = current_millis;
			}
			if (user_input_.get_digital_value(UserInput::DigitalInputList::back_button) && (current_millis - state50_last_millis_for_buttons > state50_delay_for_buttons))
			{
				state50_right_leg_lifted ? state50_right_leg_lifted = false : state50_right_leg_lifted = true;
				state50_last_millis_for_buttons = current_millis;
			}
		}
		
		
		if (state50_left_leg_lifted)
		{
			left_leg_length = left_leg_length - state50_distance_to_lift;
		}
		if (state50_right_leg_lifted)
		{
			right_leg_length = right_leg_length - state50_distance_to_lift;
		}

		// Get left leg's pitch angle setpoints from DSP kinematics
		//left_leg_length = left_leg_length - config_.kinematics.height_hip - config_.kinematics.height_ankle;
		double left_ankle_pitch_angle, right_ankle_pitch_angle;
		double left_knee_pitch_angle, right_knee_pitch_angle;
		double left_hip_pitch_angle, right_hip_pitch_angle;
		global_kinematics_.get_joint_angles_for_prismatic_length(left_leg_length, 0.0, left_ankle_pitch_angle, left_knee_pitch_angle, left_hip_pitch_angle);

		// Get right leg's pitch angle setpoints from DSP kinematics
		//right_leg_length = right_leg_length - config_.kinematics.height_hip - config_.kinematics.height_ankle;
		global_kinematics_.get_joint_angles_for_prismatic_length(right_leg_length, 0.0, right_ankle_pitch_angle, right_knee_pitch_angle, right_hip_pitch_angle);

		// Compute CM tracking control: The output is the ZMP setpoint
		Vector2d ZMP_ref_xy = cm_tracking_controller_.compute_ZMP_setpoint();

		// Apply new ZMP setpoint for X-axis ZMP tracking controllers
		left_foot_ZMP_tracking_controller_.set_setpoint_x_mm(ZMP_ref_xy(0));
		right_foot_ZMP_tracking_controller_.set_setpoint_x_mm(ZMP_ref_xy(0));

		// Apply new ZMP setpoint for X-axis ZMP tracking controllers
		left_foot_ZMP_tracking_controller_.set_setpoint_y_mm(ZMP_ref_xy(1) - state50_desired_step_width);
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

		// SIGNAL PRINTS
		
		// Prints for CM estimation testing in Y and X axis
		// Reset time for debug
		if (command_->commands.reset_time)
		{
			command_->commands.reset_time = false;
			debug_millis = millis();
		}
		Vector3d aCM = global_kinematics_.get_CoM_acceleration();
		Vector3d vCM = global_kinematics_.get_CoM_velocity();
		Vector3d CM = global_kinematics_.get_CoM_location();
		Vector2d ZMP = force_sensors_manager_.get_global_ZMP();
		String controllers_on = "";
		controllers_on += (left_foot_ZMP_tracking_controller_.is_x_on()) ? "1" : "0";
		controllers_on += (left_foot_ZMP_tracking_controller_.is_y_on()) ? "1" : "0";
		controllers_on += (right_foot_ZMP_tracking_controller_.is_x_on()) ? "1" : "0";
		controllers_on += (right_foot_ZMP_tracking_controller_.is_y_on()) ? "1" : "0";
		Serial.println("Yaxis-> timestamp, ZMPref, ZMP, CM, vCM, aCM, {lx_on ly_on rx_on ry_on}: \t" + (String)(millis() - debug_millis) + "\t" + (String)ZMP_ref_xy(1) + "\t" + (String)ZMP(1) + "\t" + (String)CM(1) + "\t" + (String)vCM(1) + "\t" + (String)aCM(1) + "\t" + controllers_on);
		//Serial.println("Xaxis-> timestamp, setpoint, CM, vCM, aCM, ZMP: \t" + (String)(millis() - debug_millis) + "\t" + (String)DSP_CM_setpoint_ + "\t" + (String)CM(0) + "\t" + (String)vCM(0) + "\t" + (String)aCM(0) + "\t" + (String)ZMP(0));

	}
}
