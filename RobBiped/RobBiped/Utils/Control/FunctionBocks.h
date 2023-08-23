/*
 * FunctionBocks.h
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

#ifndef _FUNCTIONBLOCKS_h
#define _FUNCTIONBLOCKS_h

#include "arduino.h"

namespace Control {

	void saturation(double& _input, double& _lower_limit, double& _upper_limit, double& _output);
	double saturation(double& _input, double& _lower_limit, double& _upper_limit);

	double inverse_deadband(const double &_sign_variable, const double &_u, const double &_positive_compensation, const double &_negative_compensation);

	double smoooth_inverse_deadband(const double &_u, const double _offset_for_zero, const double _positive_compensation, const double _negative_compensation);

	double two_points_interpolate(double &_input, double &_x, double &_pre_x, double &_y, double &_pre_y);

	double custom_curve_interpolation(double &_input, std::vector<double> &_x_points, std::vector<double> &_y_points);

}	// End namespace Control

#endif
