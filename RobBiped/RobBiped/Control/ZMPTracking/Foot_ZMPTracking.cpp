/*
 * Foot_ZMPTracking.cpp
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

#include "Foot_ZMPTracking.h"

void Control::Foot_ZMPTracking::assoc_config(Configuration::Configs::Control::Foot_ZMPTracking_x& _config)
{
	config_x_ = &_config;
}

void Control::Foot_ZMPTracking::assoc_config(Configuration::Configs::Control::Foot_ZMPTracking_y& _config)
{
	config_y_ = &_config;
}

void Control::Foot_ZMPTracking::configure()
{
	conf_pid_x_ = &(config_x_->pid);
	conf_pid_y_ = &(config_y_->pid);

	conf_curve_x_ = &(config_x_->feedforward_curve);
	conf_curve_y_ = &(config_y_->feedforward_curve);

	conf_db_x_ = &(config_x_->deadband_compensation);
	conf_db_y_ = &(config_y_->deadband_compensation);

	// Get Command singleton instance
	command_ = Command::get_instance();

	pid_x_.set_constants(conf_pid_x_->kp, conf_pid_x_->ki, conf_pid_x_->kd);
	double lower_saturation_radians = conf_pid_x_->lower_saturation_degrees * DEG_TO_RAD;
	double upper_saturation_radians = conf_pid_x_->upper_saturation_degrees * DEG_TO_RAD;
	pid_x_.set_saturation_constants(true, lower_saturation_radians, upper_saturation_radians);
	pid_x_.set_antiwindup(conf_pid_x_->k_windup);
	pid_x_.set_setpoint_weighting(conf_pid_x_->proportional_setpoint_weight, conf_pid_x_->derivative_setpoint_weight);

	pid_y_.set_constants(conf_pid_y_->kp, conf_pid_y_->ki, conf_pid_y_->kd);
	lower_saturation_radians = conf_pid_y_->lower_saturation_degrees * DEG_TO_RAD;
	upper_saturation_radians = conf_pid_y_->upper_saturation_degrees * DEG_TO_RAD;
	pid_y_.set_saturation_constants(true, lower_saturation_radians, upper_saturation_radians);
	pid_y_.set_antiwindup(conf_pid_y_->k_windup);
	pid_y_.set_setpoint_weighting(conf_pid_y_->proportional_setpoint_weight, conf_pid_y_->derivative_setpoint_weight);
}

void Control::Foot_ZMPTracking::set_setpoint_x_mm(double& _desired_zmp_lateral_deviation_mm)
{
	setpoint_x_mm_ = _desired_zmp_lateral_deviation_mm;
}

void Control::Foot_ZMPTracking::set_setpoint_y_mm(double& _desired_zmp_lateral_deviation_mm)
{
	setpoint_y_mm_ = _desired_zmp_lateral_deviation_mm;
}

double Control::Foot_ZMPTracking::compute_x(double& _current_foot_zmp_lateral_deviation_mm)
{
// 	double output_rad = 0.0;
// 	
// 	if (command_->commands.foot_roll_centering_debug_on)
// 	{
// 		double kp, ki, kd;
// 		pid_.get_control_action_values(kp, ki, kd);
// 		Serial.println("Action::: " + (String)output_rad + "\tKp = " + (String)kp + "\tKi = " + (String)ki + "\tKd = " + (String)kd);
// 
// 		if (command_->commands.foot_roll_centering_debug_off)
// 		{
// 			command_->commands.foot_roll_centering_debug_on = false;
// 			command_->commands.foot_roll_centering_debug_off = false;
// 		}
// 	}
// 
// 	if (!controller_on & command_->commands.foot_roll_centering_on)
// 	{
// 		controller_on = true;
// 	}
// 	if (controller_on & command_->commands.foot_roll_centering_off)
// 	{
// 		pid_.sleep();
// 		controller_on = false;
// 	}
// 
// 	if (controller_on)
// 	{
// 		pid_.compute_output(setpoint_mm_, _current_foot_zmp_lateral_deviation_mm, output_rad);
// 	}
// 
// 	return output_rad;
	return 0.0;
}

double Control::Foot_ZMPTracking::compute_y(double& _current_foot_zmp_lateral_deviation_mm)
{
	return 0.0;
}

bool Control::Foot_ZMPTracking::is_x_on()
{
	return controller_x_on;
}

bool Control::Foot_ZMPTracking::is_y_on()
{
	return controller_y_on;
}

bool Control::Foot_ZMPTracking::switch_x_on()
{
	controller_x_on = true;
}

bool Control::Foot_ZMPTracking::switch_y_on()
{
	controller_y_on = true;
}

bool Control::Foot_ZMPTracking::switch_x_off()
{
	controller_x_on = false;
}

bool Control::Foot_ZMPTracking::switch_y_off()
{
	controller_y_on = false;
}
