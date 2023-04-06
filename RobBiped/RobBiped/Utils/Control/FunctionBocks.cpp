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
