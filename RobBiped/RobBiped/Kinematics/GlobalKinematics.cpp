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

void GlobalKinematics::assoc_config(Configuration::Configs::Kinematics &_config)
{
	config_ = &_config;
}

void GlobalKinematics::init(double _centerof_right_foot, PosePhases _phase, double _desired_hip_height, double _desired_step_width)
{
	right_foot_center_ = _centerof_right_foot;
	phase_ = _phase;
	set_desired_hip_height(_desired_hip_height);
	set_desired_step_width(_desired_step_width);

	compute_lateral_DSP_home_kinematics();
}

void GlobalKinematics::set_desired_hip_height(double _desired_hip_height)
{
	desired_hip_height_ = _desired_hip_height;
}

void GlobalKinematics::set_desired_step_width(double _desired_step_width)
{
	desired_step_width_ = _desired_step_width;
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
	if (_desired_hip_center_position < right_foot_center_
		|| _desired_hip_center_position > left_foot_center_)
	{
		return false;
	}

	right_foot_roll_setpoint_ = atan2( (_desired_hip_center_position - right_foot_center_ - config_->d_hip_width / 2.0) ,
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
	return InverseKinematics::get_supporting_leg_joints_angles_from_desired_length_and_orientation(_desired_prismatic_length, _desired_forward_inclination_angle,
														config_->height_knee_ankle, config_->height_hip_knee,
														_down_joint, _mid_joint, _up_joint);
}
