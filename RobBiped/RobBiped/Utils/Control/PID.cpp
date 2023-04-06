/*
 * PID.cpp
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

#include "PID.h"

void Control::PID::set_constants(double& _Kp, double& _Ki, double& _Kd)
{
	Kp_ = _Kp;
	Ki_ = _Ki;
	Kd_ = _Kd;
}

void Control::PID::set_antiwindup(double& _Kw)
{
	Kw_ = _Kw;
}

void Control::PID::set_time_constant_millis(uint16_t& _millis)
{
	if (0 != _millis) time_constant_millis_ = _millis;
}

void Control::PID::set_saturation_constants(bool& _apply_saturation, double& _lower_limit, double& _upper_limit)
{
	apply_saturation_ = _apply_saturation;
	lower_limit_ = _lower_limit;
	upper_limit_ = _upper_limit;
}

void Control::PID::set_setpoint_weighting(double& _on_proportional, double& _on_derivative)
{
	on_proportional_setpoint_weight_ = _on_proportional;
	on_derivative_setpoint_weight_ = _on_derivative;
}

void Control::PID::compute_output(const double& _setpoint, const double& _feedback, double& _output)
{
	uint64_t current_millis_computation = millis();
	uint64_t current_time_interval = current_millis_computation - last_millis_computation;
	uint64_t time_fraction = current_time_interval / time_constant_millis_;

	// Error calculation
	double error = _setpoint - _feedback;

	// Proportional component
	double proportional_action;
	if (0.0 != Kp_)
	{
		if (0.0 != on_proportional_setpoint_weight_)
			proportional_action = Kp_ * ( on_proportional_setpoint_weight_ * _setpoint - _feedback );
		else proportional_action = Kp_ * _feedback;
	}

	// Integral component
	double integral_action;
	if (0.0 != Ki_)
	{
		double integral_action = last_integral_action + Ki_ * error * time_fraction;
		if (apply_saturation_)
		{
			double anti_windup = Kw_ * (last_saturated_controller_output - last_controller_output);
			integral_action += anti_windup;
		}
	}

	// Derivative component
	double derivative_action;
	if (0.0 != Kd_)
	{
		double diff_feedback = _feedback - last_feedback_;
		if (0.0 != on_derivative_setpoint_weight_)
		{
			double diff_setpoint = _feedback - last_feedback_;
			derivative_action = - Kd_ * ( on_derivative_setpoint_weight_ * diff_setpoint - diff_feedback ) / time_fraction;
		}
		else derivative_action = - Kd_ * diff_feedback / time_fraction;
	}
	
	// Sum of the components
	double sum = proportional_action + integral_action + derivative_action;

	// Saturation
	saturation(sum, lower_limit_, upper_limit_, _output);

	// Memory update
	last_millis_computation = current_millis_computation;
	last_setpoint_ = _setpoint;
	last_feedback_ = _feedback;
	last_integral_action = integral_action;
	last_controller_output = sum;
	last_saturated_controller_output = _output;
}
