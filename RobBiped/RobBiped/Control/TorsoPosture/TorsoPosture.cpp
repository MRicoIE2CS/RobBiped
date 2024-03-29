/*
 * TorsoPosture.cpp
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

#include "TorsoPosture.h"

void Control::TorsoPosture::assoc_config(Configuration::Configs::Control::TorsoPosture& _config)
{
	config_ = &_config;
}

void Control::TorsoPosture::init()
{
	kp_ = &(config_->kp);
	ki_ = &(config_->ki);
	kd_ = &(config_->kd);
	k_windup_ = &(config_->k_windup);
	proportional_setpoint_weight_ = &(config_->proportional_setpoint_weight);
	derivative_setpoint_weight_ = &(config_->derivative_setpoint_weight);
	negative_db_compensation = &(config_->negative_db_compensation_rad);
	positive_db_compensation = &(config_->positive_db_compensation_rad);
	lower_saturation_degrees_ = &(config_->lower_saturation_degrees);
	upper_saturation_degrees_ = &(config_->upper_saturation_degrees);
	
	// Get Command singleton instance
	command_ = Command::get_instance();
	
	pid_.set_constants(*kp_, *ki_, *kd_);
	double lower_saturation_radians = *lower_saturation_degrees_ * DEG_TO_RAD;
	double upper_saturation_radians = *upper_saturation_degrees_ * DEG_TO_RAD;
	pid_.set_saturation_constants(true, lower_saturation_radians, upper_saturation_radians);
	pid_.set_antiwindup(*k_windup_);
	pid_.set_setpoint_weighting(*proportional_setpoint_weight_, *derivative_setpoint_weight_);
	pid_.set_deadband_compensation(*negative_db_compensation, *positive_db_compensation);
	pid_.set_derivative_filter_time_constant(config_->derivative_time_constant_ms_);
}

void Control::TorsoPosture::set_setpoint_rad(double& _desired_torso_pitch_angle)
{
	setpoint_rad_ = _desired_torso_pitch_angle;
}

double Control::TorsoPosture::compute(double& _current_torso_pitch_angle_rad)
{
	double output_rad = 0.0;

	if (command_->commands.torso_posture_debug_on)
	{
		double kp, ki, kd;
		pid_.get_control_action_values(kp, ki, kd);
		Serial.println("Action::: " + (String)output_rad + "\tKp = " + (String)kp + "\tKi = " + (String)ki + "\tKd = " + (String)kd);

		if (command_->commands.torso_posture_debug_off)
		{
			command_->commands.torso_posture_debug_on = false;
			command_->commands.torso_posture_debug_off = false;
		}
	}

	if (command_->commands.torso_posture_on)
	{
		controller_on = true;

		command_->commands.torso_posture_on = false;
	}
	if (command_->commands.torso_posture_off)
	{
		pid_.sleep();
		controller_on = false;

		command_->commands.torso_posture_off = false;
	}

	if (controller_on)
	{
		pid_.compute_output(setpoint_rad_, _current_torso_pitch_angle_rad, output_rad, -_current_torso_pitch_angle_rad);
	}

	return output_rad;
}

bool Control::TorsoPosture::is_on()
{
	return controller_on;
}
