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

void Control::PID::set_time_constant_millis(uint16_t& _millis)
{
	time_constant_millis_ = _millis;
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
	
}
