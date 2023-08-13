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

void GlobalKinematics::init(double _centerof_right_foot, PosePhases _phase, double _desired_hip_height, double _desired_step_width)
{
	right_foot_center_y_ = _centerof_right_foot;
	left_foot_center_y_ = right_foot_center_y_ + _desired_step_width;
	phase_ = _phase;
	set_desired_hip_height(_desired_hip_height);
	set_desired_step_width(_desired_step_width);

	CoM_location_.set_CoM_height(_desired_hip_height + config_->height_CM_from_hip);
	CoM_location_.set_filter_complement_k(config_->Kfilter_CM_location);
	init_CoM_location();
	filter_CoM_location_.set_time_constant(100);

	compute_lateral_DSP_home_kinematics();
}

void GlobalKinematics::set_desired_hip_height(double _desired_hip_height)
{
	desired_hip_height_ = _desired_hip_height;
	CoM_location_.set_CoM_height(desired_hip_height_ - config_->height_CM_from_hip);
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

GlobalKinematics::PosePhases GlobalKinematics::get_walking_phase()
{
	return phase_;
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

bool GlobalKinematics::compute_lateral_DSP_kinematics(const double &_desired_hip_center_position)
{
	if (_desired_hip_center_position < right_foot_center_y_
		|| _desired_hip_center_position > left_foot_center_y_)
	{
		return false;
	}

	right_foot_roll_setpoint_ = atan2( (_desired_hip_center_position - right_foot_center_y_ - config_->d_hip_width / 2.0) ,
										(desired_hip_height_ - config_->height_foot) );
	left_foot_roll_setpoint_ = atan2( (desired_step_width_ - _desired_hip_center_position - config_->d_hip_width / 2.0) ,
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

bool GlobalKinematics::get_joint_angles_for_leg_length(const double &_desired_prismatic_length, const double &_desired_forward_inclination_angle,
														double &_down_joint, double &_mid_joint, double &_up_joint)
{
	// Limits for prismatic length
	// TODO: Get limits from configuration
	double desired_prismatic_length = _desired_prismatic_length;
	if (desired_prismatic_length < 115)
	{
		desired_prismatic_length = 115;
	}
	else if (desired_prismatic_length > 140)
	{
		desired_prismatic_length = 140;
	}

	// Computation
	return InverseKinematics::get_supporting_leg_joints_angles_from_desired_length_and_orientation(desired_prismatic_length, _desired_forward_inclination_angle,
														config_->height_knee_ankle, config_->height_hip_knee,
														_down_joint, _mid_joint, _up_joint);
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

double GlobalKinematics::compensate_hip_roll_angle(double &_desired_hip_roll_angle)
{
	double compensated_angle;
	double hi_x = 0.1;
	double lo_x = 0.0;
	double hi_y = 0.1;
	double lo_y = 0.0;
	if (_desired_hip_roll_angle <= lo_x) compensated_angle = _desired_hip_roll_angle + lo_y;
	else if (_desired_hip_roll_angle >= hi_x) compensated_angle = _desired_hip_roll_angle + hi_y;
	else
	{
		double slope = (hi_y - lo_y) / (hi_x - lo_x);
		compensated_angle = _desired_hip_roll_angle + (_desired_hip_roll_angle - lo_x) * slope;
	}
	return compensated_angle;
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

	double x_zmp, y_zmp;
	force_sensors_manager_->get_global_ZMP(x_zmp, y_zmp);
	Vector2d ZMP_location;
	ZMP_location(0) = x_zmp;
	ZMP_location(1) = y_zmp;

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
