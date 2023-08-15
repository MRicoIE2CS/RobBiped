/*
 * GlobalStabilization.cpp
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

#include "GlobalStabilization.h"

void Control::GlobalStabilization::assoc_config(Configuration::Configs::Control::CMTracking &_config)
{
	config_ = &_config;
}

void Control::GlobalStabilization::assoc_globalkinematics(GlobalKinematics &_global_kinematics)
{
	global_kinematics_ = &_global_kinematics;
}

void Control::GlobalStabilization::assoc_sensors(ForceSensorsManager &_force_sensors_manager)
{
	force_sensor_ = &_force_sensors_manager;
}

void Control::GlobalStabilization::init()
{
	Tra_x_ = config_->Tra_x;
	Tra_y_ = config_->Tra_y;

	d0x = config_->d0_x;
	d1x = config_->d1_x;
	d2x = config_->d2_x;
	d0y = config_->d0_y;
	d1y = config_->d1_y;
	d2y = config_->d2_y;
	
	PregeneratedTrajectory CM_path_x_;
	PregeneratedTrajectory dCM_path_x_;
	PregeneratedTrajectory ddCM_path_x_;
	PregeneratedTrajectory dddCM_path_x_;
	PregeneratedTrajectory CM_path_y_;
	PregeneratedTrajectory dCM_path_y_;
	PregeneratedTrajectory ddCM_path_y_;
	PregeneratedTrajectory dddCM_path_y_;

	CM_path_x_.set_sampling_time_ms(config_->paths_sampletime_ms);
	CM_path_x_.set_file_name(config_->CM_path_x_filename);
	CM_path_x_.init();
	dCM_path_x_.set_sampling_time_ms(config_->paths_sampletime_ms);
	dCM_path_x_.set_file_name(config_->dCM_path_x_filename);
	dCM_path_x_.init();
	ddCM_path_x_.set_sampling_time_ms(config_->paths_sampletime_ms);
	ddCM_path_x_.set_file_name(config_->ddCM_path_x_filename);
	ddCM_path_x_.init();
	dddCM_path_x_.set_sampling_time_ms(config_->paths_sampletime_ms);
	dddCM_path_x_.set_file_name(config_->dddCM_path_x_filename);
	dddCM_path_x_.init();

	CM_path_y_.set_sampling_time_ms(config_->paths_sampletime_ms);
	CM_path_y_.set_file_name(config_->CM_path_y_filename);
	CM_path_y_.init();
	dCM_path_y_.set_sampling_time_ms(config_->paths_sampletime_ms);
	dCM_path_y_.set_file_name(config_->dCM_path_y_filename);
	dCM_path_y_.init();
	ddCM_path_y_.set_sampling_time_ms(config_->paths_sampletime_ms);
	ddCM_path_y_.set_file_name(config_->ddCM_path_y_filename);
	ddCM_path_y_.init();
	dddCM_path_y_.set_sampling_time_ms(config_->paths_sampletime_ms);
	dddCM_path_y_.set_file_name(config_->dddCM_path_y_filename);
	dddCM_path_y_.init();
}

void Control::GlobalStabilization::set_mode(Mode &_mode)
{
	mode_ = _mode;
}

void Control::GlobalStabilization::reset_trajectory()
{
	is_runnning_ = false;
}

void Control::GlobalStabilization::start_trajectories()
{
	CM_path_x_.start_trajectory();
	dCM_path_x_.start_trajectory();
	ddCM_path_x_.start_trajectory();
	dddCM_path_x_.start_trajectory();
	CM_path_y_.start_trajectory();
	dCM_path_y_.start_trajectory();
	ddCM_path_y_.start_trajectory();
	dddCM_path_y_.start_trajectory();
}

void Control::GlobalStabilization::get_reference_signals(Vector2d &_CM_ref, Vector2d &_vCM_ref, Vector2d &_aCM_ref, Vector2d &_jCM_ref)
{
	_CM_ref(0) = CM_path_x_.get_value();
	_CM_ref(1) = CM_path_y_.get_value();
	_vCM_ref(0) = dCM_path_x_.get_value();
	_vCM_ref(1) = dCM_path_y_.get_value();
	_aCM_ref(0) = ddCM_path_x_.get_value();
	_aCM_ref(1) = ddCM_path_y_.get_value();
	_jCM_ref(0) = dddCM_path_x_.get_value();
	_jCM_ref(1) = dddCM_path_y_.get_value();
}

void Control::GlobalStabilization::get_feedback_signals(Vector2d &_CM_est, Vector2d &_vCM_est, Vector2d &_aCM_med, Vector2d &_ZMP_med)
{
	Vector3d CoM_location_xyz = global_kinematics_->get_CoM_location();
	_CM_est(0) = CoM_location_xyz(0);
	_CM_est(1) = CoM_location_xyz(1);
	h_ = CoM_location_xyz(2);
	Vector3d CoM_velocity_xyz = global_kinematics_->get_CoM_velocity();
	_vCM_est(0) = CoM_velocity_xyz(0);
	_vCM_est(1) = CoM_velocity_xyz(1);
	Vector3d CoM_acceleration_xyz = global_kinematics_->get_CoM_acceleration();
	_aCM_med(0) = CoM_acceleration_xyz(0);
	_aCM_med(1) = CoM_acceleration_xyz(1);
	Vector2d ZMP_xy = force_sensor_->get_global_ZMP();
	_ZMP_med(0) = ZMP_xy(0);
	_ZMP_med(1) = ZMP_xy(1);
}

Vector2d Control::GlobalStabilization::compute_ZMP_action()
{
	if (!is_runnning_)
	{
		is_runnning_ = true;
		start_trajectories();
	}

	// References for CM position, velocity, acceleration and jerk
	Vector2d CM_ref, vCM_ref, aCM_ref, jCM_ref;
	get_reference_signals(CM_ref, vCM_ref, aCM_ref, jCM_ref);

	// Feedback of the CM position, velocity and acceleration, and ZMP position
	Vector2d CM_est, vCM_est, aCM_med, ZMP_med;
	get_feedback_signals(CM_est, vCM_est, aCM_med, ZMP_med);

	// Error computation - x
	double err_x = CM_ref(0) - CM_est(0);
	double v_err_x = vCM_ref(0) - vCM_est(0);
	double a_err_x = aCM_ref(0) - aCM_med(0);
	// Error computation - y
	double err_y = CM_ref(1) - CM_est(1);
	double v_err_y = vCM_ref(1) - vCM_est(1);
	double a_err_y = aCM_ref(1) - aCM_med(1);

	// v computation
	double v_x = d0x * err_x + d1x * v_err_x + d2x * a_err_x + jCM_ref(0);
	double v_y = d0y * err_y + d1y * v_err_y + d2y * a_err_y + jCM_ref(1);

	// u computation
	double u_x = (Tra_x_ * h_ / g_) * ((g_/h_) * vCM_est(0) + (g_/(Tra_x_*h_) * ZMP_med(0) - v_x));
	double u_y = (Tra_y_ * h_ / g_) * ((g_/h_) * vCM_est(1) + (g_/(Tra_y_*h_) * ZMP_med(1) - v_y));

	control_action_(0) = u_x;
	control_action_(1) = u_y;
	return control_action_;
}

Vector2d Control::GlobalStabilization::get_ZMP_action()
{
	return control_action_;
}

