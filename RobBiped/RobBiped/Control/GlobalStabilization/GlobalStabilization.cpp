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

void Control::GlobalStabilization::assoc_sensors(ForceSensorsManager &_force_sensors_manager, GyroscopeAccelerometerManager &_gyroscope_accelerometer_manager)
{
	gyroacc_sensor_ = &_gyroscope_accelerometer_manager;
	force_sensor_ = &_force_sensors_manager;
}

void Control::GlobalStabilization::set_mode(Mode &_mode)
{
	mode_ = _mode;
}

void Control::GlobalStabilization::reset_trajectory()
{
	is_runnning_ = false;
}

void Control::GlobalStabilization::get_all_signals(double _reference_signals_x[4], double _reference_signals_y[4], double _feedback_signals_x[4], double _feedback_signals_y[4])
{
	// Reference signals
	

	// Feedback signals
	Vector3d CoM_location_xyz = global_kinematics_->get_CoM_location();
	_feedback_signals_x[0] = CoM_location_xyz(0);
	_feedback_signals_y[0] = CoM_location_xyz(1);
	h_ = CoM_location_xyz(2);
	Vector3d CoM_velocity_xyz = global_kinematics_->get_CoM_velocity();
	_feedback_signals_x[1] = CoM_velocity_xyz(0);
	_feedback_signals_y[1] = CoM_velocity_xyz(1);
	Vector3d CoM_acceleration_xyz = global_kinematics_->get_CoM_acceleration();
	_feedback_signals_x[2] = CoM_acceleration_xyz(0);
	_feedback_signals_y[2] = CoM_acceleration_xyz(1);
	Vector3d ZMP_xy = force_sensor_->get_global_ZMP()
	_feedback_signals_x[3] = ZMP_xy(0);
	_feedback_signals_y[3] = ZMP_xy(1);
}

Vector2d Control::GlobalStabilization::compute_ZMP_action()
{
	// Feedback of the CM position, velocity and acceleration, and ZMP position
	Vector2d CM_est, vCM_est, aCM_med, ZMP_med;
	double feedback_signals_x[4];
	double feedback_signals_y[4];
	// References for CM position, velocity, acceleration and jerk
	Vector2d CM_ref, vCM_ref, aCM_ref, jCM_ref;
	double reference_signals_x[4];
	double reference_signals_y[4];
	// Obtain of all the signals
	get_all_signals(reference_signals_x, reference_signals_y, feedback_signals_x, feedback_signals_y);
	CM_est(0) = feedback_signals_x[0];
	CM_est(1) = feedback_signals_y[0];
	vCM_est(0) = feedback_signals_x[1];
	vCM_est(1) = feedback_signals_y[1];
	aCM_med(0) = feedback_signals_x[2];
	aCM_med(1) = feedback_signals_y[2];
	ZMP_med(0) = feedback_signals_x[3];
	ZMP_med(1) = feedback_signals_y[3];
	CM_ref(0) = reference_signals_x[0];
	CM_ref(1) = reference_signals_y[0];
	vCM_ref(0) = reference_signals_x[1];
	vCM_ref(1) = reference_signals_y[1];
	aCM_ref(0) = reference_signals_x[2];
	aCM_ref(1) = reference_signals_y[2];
	jCM_ref(0) = reference_signals_x[3];
	jCM_ref(1) = reference_signals_y[3];

	// Error computation
	
	// v computation
	
	// u computation
	

	return control_action_;
}

Vector2d Control::GlobalStabilization::get_ZMP_action()
{
	return control_action_;
}

