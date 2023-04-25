/*
 * TrayectoryInterpolator.cpp
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

#include "TrayectoryInterpolator.h"

bool Control::LinearTrajectoryInterpolator::configure_trayectory(const double& _initial_value, const double& _target, const uint64_t& _transition_time_ms)
{
	if ((_target == _initial_value)
		|| (0 == _transition_time_ms)
		)
	{
		return false;
	}
	target_ = _target;
	initial_value_ = _initial_value;
	transition_time_ms_ = _transition_time_ms;
	return true;
}

bool Control::LinearTrajectoryInterpolator::compute_output(double& _output)
{
	if (!initiated_)
	{
		initial_millis_ = millis();
		slope_ = (target_ - initial_value_) / (transition_time_ms_);
		_output = initial_value_;

		initiated_ = true;
		return true;
	}
	else
	{
		double calculated_value = initial_value_ + slope_ * (millis() - initial_millis_);

		if (is_target_reached(calculated_value))
		{
			_output = target_;
			return false;
		}
		else
		{
			_output = calculated_value;
			return true;
		}
	}
}

bool Control::LinearTrajectoryInterpolator::is_target_reached(double& _calculated_value)
{
	if (((0 < slope_) && (_calculated_value > target_))
	   || ((0 > slope_) && (_calculated_value < target_)))
	{
		return true;
	}
	else return false;
}

void Control::LinearTrajectoryInterpolator::reset()
{
	initiated_ = false;
}
