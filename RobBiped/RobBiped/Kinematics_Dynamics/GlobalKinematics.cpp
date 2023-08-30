/*
 * GlobalKinematics.cpp
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

#include "GlobalKinematics.h"

#include "../Sensors/GyroscopeAccelerometerManager.h"
#include "../Sensors/ForceSensorsManager.h"
#include "../Utils/Control/FunctionBocks.h"
#include "../Utils/Kinematics/Geometry/Rotation.h"

void GlobalKinematics::assoc_config(Configuration::Configs::Kinematics &_config)
{
	config_ = &_config;
}

void GlobalKinematics::assoc_sensors(ForceSensorsManager &_force_sensors_manager, GyroscopeAccelerometerManager &_gyroscope_accelerometer_manager)
{
	force_sensors_manager_ = &_force_sensors_manager;
	gyroscope_accelerometer_manager_ = &_gyroscope_accelerometer_manager;
}

void GlobalKinematics::init(double _centerof_right_foot, WalkingPhase _phase, double _desired_hip_height, double _desired_step_width)
{
	right_foot_center_y_ = _centerof_right_foot;
	left_foot_center_y_ = right_foot_center_y_ + _desired_step_width;
	phase_ = _phase;
	set_desired_hip_height(_desired_hip_height);
	set_desired_step_width(_desired_step_width);

	CoM_location_.set_CoM_height(_desired_hip_height + config_->height_CM_from_hip);
	CoM_location_.set_filter_complement_k(config_->Kfilter_CM_location, config_->Kfilter_CM_velocity);
	init_CoM_location();
	filter_CoM_location_.set_time_constant(100);

	compute_lateral_DSP_home_kinematics();
}

bool GlobalKinematics::set_desired_hip_height(double _desired_hip_height)
{
	// Limit for hip heigth
	bool retcode_OK = true;
	double desired_hip_height = _desired_hip_height;
	if (_desired_hip_height < config_->limit_down_hip_height)
	{
		Serial.println("GlobalKinematics: Lower Limit for hip height!");
		desired_hip_height = config_->limit_down_hip_height;
		retcode_OK = false;
	}
	else if (_desired_hip_height > config_->limit_up_hip_height)
	{
		Serial.println("GlobalKinematics: Upper Limit for hip height!");
		desired_hip_height = config_->limit_up_hip_height;
		retcode_OK = false;
	}
	desired_hip_height_ = desired_hip_height;

	CoM_location_.set_CoM_height(desired_hip_height_ + config_->height_CM_from_hip);
	return retcode_OK;
}

void GlobalKinematics::set_desired_step_width(double _desired_step_width)
{
	desired_step_width_ = _desired_step_width;
	left_foot_center_y_ = right_foot_center_y_ + _desired_step_width;
}

void GlobalKinematics::compute_lateral_DSP_home_kinematics()
{
	home_roll_angle_ = atan2( (desired_step_width_ - config_->d_hip_width) / 2.0 , (desired_hip_height_ - config_->height_foot) );

	home_leg_length_ = (desired_hip_height_ - config_->height_foot) / (cos(home_roll_angle_));
}

double GlobalKinematics::get_home_roll_angle()
{
	return home_roll_angle_;
}

double GlobalKinematics::get_home_leg_lengths()
{
	return home_leg_length_;
}

double GlobalKinematics::get_step_width()
{
	return desired_step_width_;
}

bool GlobalKinematics::compute_lateral_DSP_kinematics(const double _desired_hip_center_position)
{
// 	if (_desired_hip_center_position < right_foot_center_y_
// 		|| _desired_hip_center_position > left_foot_center_y_)
// 	{
// 		return false;
// 	}

	// TODO: Verification to avoid non reachable postures, depending on configuration, desired hip height and desired step width

	right_foot_roll_setpoint_ = atan2( (_desired_hip_center_position - right_foot_center_y_ - config_->d_hip_width / 2.0) ,
										(desired_hip_height_ - config_->height_foot) );
	left_foot_roll_setpoint_ = atan2( (desired_step_width_ - _desired_hip_center_position - right_foot_center_y_ - config_->d_hip_width / 2.0) ,
										(desired_hip_height_ - config_->height_foot) );
	
	left_leg_length_setpoint_ = (desired_hip_height_ - config_->height_foot) / cos(left_foot_roll_setpoint_);
	right_leg_length_setpoint_ = (desired_hip_height_ - config_->height_foot) / cos(right_foot_roll_setpoint_);

	return true;
}

void GlobalKinematics::get_computed_angles(double &_left_foot_roll_setpoint, double &_right_foot_roll_setpoint)
{
	_left_foot_roll_setpoint = left_foot_roll_setpoint_;
	_right_foot_roll_setpoint = right_foot_roll_setpoint_;
}

void GlobalKinematics::get_computed_leg_lengths(double &_left_leg_length_setpoint, double &_right_leg_length_setpoint)
{
	_left_leg_length_setpoint = left_leg_length_setpoint_;
	_right_leg_length_setpoint = right_leg_length_setpoint_;
}

bool GlobalKinematics::get_computed_prismatic_lengths(double &_left_prismatic_length_setpoint, double &_right_prismatic_length_setpoint)
{
	bool retcode1 = get_prismatic_lenght(left_leg_length_setpoint_, _left_prismatic_length_setpoint);
	bool retcode2 = get_prismatic_lenght(right_leg_length_setpoint_, _right_prismatic_length_setpoint);
	if (retcode1 && retcode2) return true;
	else return false;
}

bool GlobalKinematics::get_prismatic_lenght(const double &_desired_leg_length, double &_desired_prismatic_length)
{
	bool retcode_OK = true;
	// Limits for leg length

	double desired_leg_length = _desired_leg_length;
	if (_desired_leg_length < config_->limit_down_hip_height - config_->height_foot)
	{
		Serial.println("GlobalKinematics: Lower Limit for leg length!");
		desired_leg_length = config_->limit_down_hip_height - config_->height_foot;
		retcode_OK = false;
	}
	else if (_desired_leg_length > config_->limit_up_hip_height - config_->height_foot)
	{
		Serial.println("GlobalKinematics: Upper Limit for leg length!");
		desired_leg_length = config_->limit_up_hip_height - config_->height_foot;
		retcode_OK = false;
	}

	_desired_prismatic_length = desired_leg_length - config_->height_hip - config_->height_ankle;
	return retcode_OK;
}

bool GlobalKinematics::get_joint_angles_for_prismatic_length(const double &_desired_prismatic_length, const double &_desired_forward_inclination_angle,
														double &_down_joint, double &_mid_joint, double &_up_joint)
{
	bool retcode_OK = true;
	// Limits for prismatic length
	double desired_prismatic_length = _desired_prismatic_length;
	if (desired_prismatic_length < config_->limit_down_hip_height - config_->height_hip - config_->height_ankle - config_->height_foot)
	{
		Serial.println("GlobalKinematics: Lower Limit for prismatic length!");
		desired_prismatic_length = config_->limit_down_hip_height - config_->height_hip - config_->height_ankle - config_->height_foot;
		retcode_OK = false;
	}
	else if (desired_prismatic_length > config_->limit_up_hip_height - config_->height_hip - config_->height_ankle - config_->height_foot)
	{
		Serial.println("GlobalKinematics: Upper Limit for prismatic length!");
		desired_prismatic_length = config_->limit_up_hip_height - config_->height_hip - config_->height_ankle - config_->height_foot;
		retcode_OK = false;
	}

	// Computation
	bool retcode_OK2 = InverseKinematics::get_supporting_leg_joints_angles_from_desired_length_and_orientation(desired_prismatic_length, _desired_forward_inclination_angle,
														config_->height_knee_ankle, config_->height_hip_knee,
														_down_joint, _mid_joint, _up_joint);
	return (retcode_OK && retcode_OK2);
}

void GlobalKinematics::get_feet_distance(double &_frontal_distance, double &_lateral_distance)
{
	_frontal_distance = left_foot_center_x_ - right_foot_center_x_;
	_lateral_distance = left_foot_center_y_ - right_foot_center_y_;
}

void GlobalKinematics::get_right_foot_coordinates(double &_x, double &_y)
{
	_x = right_foot_center_x_;
	_y = right_foot_center_y_;
}

void GlobalKinematics::get_left_foot_coordinates(double &_x, double &_y)
{
	_x = left_foot_center_x_;
	_y = left_foot_center_y_;
}

double GlobalKinematics::compensate_hip_roll_angle(double _desired_hip_roll_angle, bool _left_or_right)
{
	double positive_compensation;
	double negative_compensation;
	if (false == _left_or_right)
	{
		positive_compensation = config_->left_hip_roll_compensation.positive_compensation;
		negative_compensation = config_->left_hip_roll_compensation.negative_compensation;
	}
	else
	{
		positive_compensation = config_->right_hip_roll_compensation.positive_compensation;
		negative_compensation = config_->right_hip_roll_compensation.negative_compensation;
	}
	
	return Control::smoooth_inverse_deadband(
					_desired_hip_roll_angle, 0.0,
					positive_compensation, negative_compensation);
}

Vector3d GlobalKinematics::correct_acceleration_inclination(const Vector3d &_CoM_acceleration_measurements_xyz, const Vector2d &_CoM_inclinations)
{
	// For the rotation of the roll (inclination in the yz plane), the sign must be inversed to comply with right hand convention.
	Vector3d corrected_CoM_acceleration_xyz = Geometry::rotate_in_xy(_CoM_acceleration_measurements_xyz, - _CoM_inclinations(1), _CoM_inclinations(0));

	return corrected_CoM_acceleration_xyz;
}

void GlobalKinematics::init_CoM_location()
{
	double x_zmp, y_zmp;
	force_sensors_manager_->get_global_ZMP(x_zmp, y_zmp);
	Vector3d ZMP_location;
	ZMP_location(0) = x_zmp;
	ZMP_location(1) = y_zmp;
	ZMP_location(2) = desired_hip_height_ + config_->height_CM_from_hip;
	CoM_location_.init_location(ZMP_location);
}

Vector3d GlobalKinematics::compute_CoM_location()
{

	float incl_pitch, incl_roll;
	gyroscope_accelerometer_manager_->get_value_angle_z_pitch_rad(incl_pitch);
	gyroscope_accelerometer_manager_->get_value_angle_z_roll_rad(incl_roll);
	Vector2d inclination;
	inclination(0) = incl_pitch;
	inclination(1) = incl_roll;

	float ax, ay, az;
	gyroscope_accelerometer_manager_->get_filtered_acc_values(ax, ay, az);
	Vector3d CoM_measured_accelerations;
	CoM_measured_accelerations(0) = ax;
	CoM_measured_accelerations(1) = ay;
	CoM_measured_accelerations(2) = az;

	Vector3d CoM_corrected_accelerations = correct_acceleration_inclination(CoM_measured_accelerations, inclination);

	Vector2d ZMP_location = force_sensors_manager_->get_global_ZMP();

	Vector3d CoM_location;
	CoM_location = CoM_location_.compute_location(CoM_corrected_accelerations, ZMP_location);

// // _____ SIGNAL RECORD

// 		Serial.println("CoM_fCoM_ZMP_y / accy_aCM_y: \t" + (String)filter_CoM_location_.filter(CoM_location(1)) + "\t" + (String)ZMP_location(1) + "\t" + (String)CoM_corrected_accelerations(1));
//Serial.println("accy_aCM_y: \t" + (String)CoM_measured_accelerations(1) + "\t" + (String)CoM_corrected_accelerations(1) + "\t" + (String)incl_pitch + "\t" + (String)incl_roll);
//Serial.println("accy_aCM_y: \t" + (String)CoM_corrected_accelerations(0) + "\t" + (String)CoM_corrected_accelerations(1) + "\t" + (String)CoM_corrected_accelerations(2) + "\t" + (String)incl_pitch + "\t" + (String)incl_roll);
// // _____suposed_com_location_

	return CoM_location;
}

Vector3d GlobalKinematics::get_CoM_location()
{
	return CoM_location_.get_location();
}

Vector3d GlobalKinematics::get_CoM_velocity()
{
	return CoM_location_.get_velocity();
}

Vector3d GlobalKinematics::get_CoM_acceleration()
{
	return CoM_location_.get_acceleration();
}

bool GlobalKinematics::check_walking_phase()
{
	has_there_been_a_phase_change_ = false;

	if (WalkingPhase::DSP_left == phase_)
	{
		bool transition_condition = is_zmp_over_left_footprint()/* && is_com_over_left_footprint()*/;
		if (transition_condition)
		{
			phase_ = WalkingPhase::SSP_left;
			has_right_foot_been_lifted = false;
			has_there_been_a_phase_change_ = true;
		}
	}
	else if (WalkingPhase::DSP_right == phase_)
	{
		bool transition_condition = is_zmp_over_right_footprint()/* && is_com_over_right_footprint()*/;
		if (transition_condition)
		{
			phase_ = WalkingPhase::SSP_right;
			has_left_foot_been_lifted = false;
			has_there_been_a_phase_change_ = true;
		}
	}
	else if (WalkingPhase::SSP_left == phase_)
	{
		if (!has_right_foot_been_lifted) has_right_foot_been_lifted = !force_sensors_manager_->is_right_foot_touching_ground();

		bool transition_condition = has_right_foot_been_lifted && force_sensors_manager_->is_right_foot_touching_ground() && lifting_maneuver_performed;
		if (transition_condition)
		{
			phase_ = WalkingPhase::DSP_right;
			has_right_foot_been_lifted = false;
			has_there_been_a_phase_change_ = true;
		}
	}
	else if (WalkingPhase::SSP_right == phase_)
	{
		if (!has_left_foot_been_lifted) has_left_foot_been_lifted = !force_sensors_manager_->is_left_foot_touching_ground();

		bool transition_condition = has_left_foot_been_lifted && force_sensors_manager_->is_left_foot_touching_ground() && lifting_maneuver_performed;
		if (transition_condition)
		{
			phase_ = WalkingPhase::DSP_left;
			has_left_foot_been_lifted = false;
			has_there_been_a_phase_change_ = true;
		}
	}

	return has_there_been_a_phase_change_;
}

GlobalKinematics::WalkingPhase GlobalKinematics::get_current_walking_phase()
{
	return phase_;
}

void GlobalKinematics::force_current_walking_phase(WalkingPhase _phase)
{
	if (WalkingPhase::DSP_left == _phase)
	{
		phase_ = WalkingPhase::DSP_left;
		has_left_foot_been_lifted = false;
		has_there_been_a_phase_change_ = true;
	}
	else if (WalkingPhase::DSP_right == _phase)
	{
		has_right_foot_been_lifted = false;
		phase_ = WalkingPhase::DSP_right;
		has_there_been_a_phase_change_ = true;
	}
	else if (WalkingPhase::SSP_left == _phase)
	{
		phase_ = WalkingPhase::SSP_left;
		has_right_foot_been_lifted = false;
		has_there_been_a_phase_change_ = true;
	}
	else if (WalkingPhase::SSP_right == _phase)
	{
		phase_ = WalkingPhase::SSP_right;
		has_left_foot_been_lifted = false;
		has_there_been_a_phase_change_ = true;
	}
}

bool GlobalKinematics::is_zmp_over_left_footprint()
{
	Vector2d ZMP_location = force_sensors_manager_->get_global_ZMP();
	
	double limit_x_min = left_foot_center_x_ - (config_->feet_dimensions.frontBack_separation / 2);
	double limit_x_max = left_foot_center_x_ + (config_->feet_dimensions.frontBack_separation / 2);

	bool within_x_limits = (ZMP_location(0) > limit_x_min) && (ZMP_location(0) < limit_x_max);

	double limit_y_min = left_foot_center_y_ - (config_->feet_dimensions.leftRight_separation / 2);
	double limit_y_max = left_foot_center_y_ + (config_->feet_dimensions.leftRight_separation / 2);

	bool within_y_limits = (ZMP_location(1) > limit_y_min) && (ZMP_location(1) < limit_y_max);

	return within_x_limits && within_y_limits;
}

bool GlobalKinematics::is_zmp_over_right_footprint()
{
	Vector2d ZMP_location = force_sensors_manager_->get_global_ZMP();
	
	double limit_x_min = right_foot_center_x_ - (config_->feet_dimensions.frontBack_separation / 2);
	double limit_x_max = right_foot_center_x_ + (config_->feet_dimensions.frontBack_separation / 2);

	bool within_x_limits = (ZMP_location(0) > limit_x_min) && (ZMP_location(0) < limit_x_max);

	double limit_y_min = right_foot_center_y_ - (config_->feet_dimensions.leftRight_separation / 2);
	double limit_y_max = right_foot_center_y_ + (config_->feet_dimensions.leftRight_separation / 2);

	bool within_y_limits = (ZMP_location(1) > limit_y_min) && (ZMP_location(1) < limit_y_max);

	
	return within_x_limits && within_y_limits;
}

// bool GlobalKinematics::is_com_over_left_footprint()
// {
// 	Vector2d ZMP_location = force_sensors_manager_->get_global_ZMP();
// 	
// 	double limit_x_min = left_foot_center_x_ - (config_->feet_dimensions.frontBack_separation / 2);
// 	double limit_x_max = left_foot_center_x_ + (config_->feet_dimensions.frontBack_separation / 2);
// 
// 	bool within_x_limits = (ZMP_location(0) > limit_x_min) && (ZMP_location(0) < limit_x_max);
// 
// 	double limit_y_min = left_foot_center_y_ - (config_->feet_dimensions.leftRight_separation / 2);
// 	double limit_y_max = left_foot_center_y_ + (config_->feet_dimensions.leftRight_separation / 2);
// 
// 	bool within_y_limits = (ZMP_location(1) > limit_y_min) && (ZMP_location(1) < limit_y_max);
// 
// 	return within_x_limits && within_y_limits;
// }
// 
// bool GlobalKinematics::is_com_over_right_footprint()
// {
// 	Vector2d ZMP_location = force_sensors_manager_->get_global_ZMP();
// 	
// 	double limit_x_min = right_foot_center_x_ - (config_->feet_dimensions.frontBack_separation / 2);
// 	double limit_x_max = right_foot_center_x_ + (config_->feet_dimensions.frontBack_separation / 2);
// 
// 	bool within_x_limits = (ZMP_location(0) > limit_x_min) && (ZMP_location(0) < limit_x_max);
// 
// 	double limit_y_min = right_foot_center_y_ - (config_->feet_dimensions.leftRight_separation / 2);
// 	double limit_y_max = right_foot_center_y_ + (config_->feet_dimensions.leftRight_separation / 2);
// 
// 	bool within_y_limits = (ZMP_location(1) > limit_y_min) && (ZMP_location(1) < limit_y_max);
// 
// 	
// 	return within_x_limits && within_y_limits;
// }

bool GlobalKinematics::has_there_been_a_phase_change()
{
	return has_there_been_a_phase_change_;
}
