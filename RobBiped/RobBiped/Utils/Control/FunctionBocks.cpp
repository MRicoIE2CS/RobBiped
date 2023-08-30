/*
 * FunctionBocks.cpp
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

#include "FunctionBocks.h"

void Control::saturation(double& _input, double& _lower_limit, double& _upper_limit, double& _output)
{
	if (_input < _lower_limit) _output = _lower_limit;
	else if (_input > _upper_limit) _output = _upper_limit;
	else _output = _input;
}

double Control::saturation(double& _input, double& _lower_limit, double& _upper_limit)
{
	double _output;
	if (_input < _lower_limit) _output = _lower_limit;
	else if (_input > _upper_limit) _output = _upper_limit;
	else _output = _input;
}

double Control::inverse_deadband(const double &_sign_variable, const double &_u, const double &_positive_compensation, const double &_negative_compensation)
{
	if (0.0 == _negative_compensation && 0.0 == _positive_compensation) return _u;

	if (_sign_variable > 0.0) return _u + _positive_compensation;
	else if (_sign_variable < 0.0) return _u + _negative_compensation;
	else return _u;
}

double Control::smoooth_inverse_deadband(const double &_u, const double _offset_for_zero, const double _positive_compensation, const double _negative_compensation)
{
	if (0.0 == _negative_compensation && 0.0 == _positive_compensation) return _u;

	if ((_u + _offset_for_zero) > 0.0)
	{
		if ((_u + _offset_for_zero) < _positive_compensation) return _u + (_u + _offset_for_zero);
		else return _u + _positive_compensation;
	}
	else if ((_u + _offset_for_zero) < 0.0)
	{
		if ((_u + _offset_for_zero) > _negative_compensation) return _u + (_u + _offset_for_zero);
		else return _u + _negative_compensation;
	}
	else return _u;
}

double Control::two_points_interpolate(double &_input, double &_x, double &_pre_x, double &_y, double &_pre_y)
{
	if (0.0 == _x - _pre_x) return 0.0;	// Avoid division by 0
	double slope = (_y - _pre_y) / (_x - _pre_x);
	return _pre_y + slope * (_input - _pre_x);
}

double Control::custom_curve_interpolation(double &_input, std::vector<double> &_ordered_x_points, std::vector<double> &_ordered_y_points)
{
	std::vector<double>::iterator it_y = _ordered_y_points.begin();
	for (std::vector<double>::iterator it_x = _ordered_x_points.begin(); it_x != _ordered_x_points.end(); ++it_x, ++it_y)
	{
		if (_input < *it_x)
		{
			if (it_x == _ordered_x_points.begin()) return *it_y;	// _input is lower than first point
			else													// _input is in between this and the last point
			{
				return Control::two_points_interpolate(_input, *it_x, *(it_x-1), *it_y, *(it_y-1));
			}
		}
		else if (_input > *it_x)
		{
			if (it_x == (_ordered_x_points.end()-1) ) return *it_y;	// _input is greater than last point
			else continue;											// _input is greater than current point
		}
		else return *it_y;
	}
}
