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

#include "../../Utils/Control/FunctionBocks.h"

void Control::Foot_ZMPTracking::assoc_config(Configuration::Configs::Control::ZMPTracking_x& _config)
{
	config_x_ = &_config;
}

void Control::Foot_ZMPTracking::assoc_config(Configuration::Configs::Control::ZMPTracking_y& _config)
{
	config_y_ = &_config;
}

void Control::Foot_ZMPTracking::configure(Foot _foot)
{
	conf_pid_x_ = &(config_x_->pid);
	conf_pid_y_ = &(config_y_->pid);

	conf_curve_x_ = &(config_x_->feedforward_curve);
	conf_curve_y_ = &(config_y_->feedforward_curve);

	if (Foot::Left == _foot) conf_db_x_ = &(config_x_->leftfoot_deadband_compensation);
	else if (Foot::Right == _foot) conf_db_x_ = &(config_x_->rightfoot_deadband_compensation);

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

Vector2d Control::Foot_ZMPTracking::compute(double& _x_zmp_feedback, double& _y_zmp_feedback)
{
	if (command_->commands.zmp_tracking_deb_toggle)
	{
		// TODO: Prints for debug
// 		double kp, ki, kd;
// 		pid_.get_control_action_values(kp, ki, kd);
//		Serial.println("Action::: " + (String)output_rad + "\tKp = " + (String)kp + "\tKi = " + (String)ki + "\tKd = " + (String)kd);
	}

	if (!controller_x_on & command_->commands.zmp_xtracking_toggle) switch_x_on();
	if (controller_x_on & !command_->commands.zmp_xtracking_toggle) switch_x_off();
	if (!controller_y_on & command_->commands.zmp_ytracking_toggle) switch_y_on();
	if (controller_y_on & !command_->commands.zmp_ytracking_toggle) switch_y_off();

	if (controller_x_on)
	{
		output_rad(0) = compute_x(_x_zmp_feedback);
	}
	if (controller_y_on)
	{
		output_rad(1) = compute_y(_x_zmp_feedback);
	}

	return output_rad;
}

double Control::Foot_ZMPTracking::compute_x(double& _x_zmp_feedback)
{
	double branch1 = Control::custom_curve_interpolation(setpoint_x_mm_, conf_curve_x_->curve_points_x, conf_curve_x_->curve_points_y);
	double branch1_ = Control::inverse_deadband(-(setpoint_x_mm_ - conf_db_x_->db_delimiting_value), branch1, conf_db_x_->positive_db_compensation_rad, conf_db_x_->negative_db_compensation_rad);
	
	double branch2 = - pid_x_.compute_output(setpoint_x_mm_, _x_zmp_feedback);
	
	double out = branch1_ + branch2;
	//Serial.println("ZMPref_x, ZMPl, lAct, ZMPr, rAct: \t" + (String)setpoint_x_mm_ + "\t" + (String)_x_zmp_feedback + "\t" + (String)branch1 + "\t" + (String)branch1_ + "\t" + (String)branch2);
	return out;
}

double Control::Foot_ZMPTracking::compute_y(double& _y_zmp_feedback)
{
	double branch1 = Control::custom_curve_interpolation(setpoint_y_mm_, conf_curve_y_->curve_points_x, conf_curve_y_->curve_points_y);
	branch1 = Control::inverse_deadband(-(_y_zmp_feedback - conf_db_y_->db_delimiting_value), branch1, conf_db_y_->positive_db_compensation_rad, conf_db_y_->negative_db_compensation_rad);
	
	double branch2 = pid_y_.compute_output(setpoint_y_mm_, _y_zmp_feedback);
	
	double out = branch1 + branch2;
	return out;
}

Vector2d Control::Foot_ZMPTracking::get_control_action()
{
	return output_rad;
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
	pid_x_.sleep();
}

bool Control::Foot_ZMPTracking::switch_y_off()
{
	controller_y_on = false;
	pid_y_.sleep();
}
