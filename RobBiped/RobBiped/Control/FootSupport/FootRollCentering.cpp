/*
 * FootRollCentering.cpp
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

#include "FootRollCentering.h"

void Control::FootRollCentering::assoc_config(Configuration::Configs::Control::FootRollCentering& _config)
{
	config_ = &_config;
}

void Control::FootRollCentering::init()
{
	kp_ = &(config_->kp);
	ki_ = &(config_->ki);
	kd_ = &(config_->kd);
	k_windup_ = &(config_->k_windup);
	proportional_setpoint_weight_ = &(config_->proportional_setpoint_weight);
	derivative_setpoint_weight_ = &(config_->derivative_setpoint_weight);
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
}

void Control::FootRollCentering::set_setpoint_rad(double& _desired_foot_roll_angle)
{
	setpoint_rad_ = _desired_foot_roll_angle;
}

double Control::FootRollCentering::compute(double& _current_foot_zmp_lateral_deviation_mm)
{
	double output_rad;
	pid_.compute_output(setpoint_rad_, _current_foot_zmp_lateral_deviation_mm, output_rad);
	
	if (command_->commands.foot_roll_centering_debug_on)
	{
		double kp, ki, kd;
		pid_.get_control_action_values(kp, ki, kd);
		Serial.println("Action::: " + (String)output_rad + "\tKp = " + (String)kp + "\tKi = " + (String)ki + "\tKd = " + (String)kd);

		if (command_->commands.foot_roll_centering_debug_off)
		{
			command_->commands.foot_roll_centering_debug_on = false;
			command_->commands.foot_roll_centering_debug_off = false;
		}
	}
	
	return output_rad;
}
