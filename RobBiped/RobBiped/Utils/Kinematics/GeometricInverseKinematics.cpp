/*
 * GeometricInverseKinematics.cpp
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

#include "GeometricInverseKinematics.h"

#include <math.h>

bool InverseKinematics::Geometric::sagittal_two_links_inverse_kinematics(
		const Vector3d &_desired_position,
		const Vector2d &_links_lengths,
		double &_target_angle_rad_1,
		double &_target_angle_rad_2,
		const bool invert_knee_side)
{
	double aux_val1 = _desired_position(1);
	double aux_val2 = _desired_position(0);
	if (0 == aux_val1 & 0 == aux_val2) return false;	// atan2 is undefined if both arguments == 0. This implies non reachable desired position.

	double C_q2 = (pow(_desired_position(0), 2) + pow(_desired_position(1), 2) - pow(_links_lengths(0), 2) - pow(_links_lengths(1), 2) ) / (2 * _links_lengths(0) * _links_lengths(1));

	aux_val1 = pow(C_q2, 2);
	if (1 < aux_val1) return false;	// S_q2 could not be computed. This implies non reachable desired position.

	double S_q2 = sqrt(1 - aux_val1);
	if (invert_knee_side) S_q2 = -S_q2;

	if (0 == S_q2 & 0 == C_q2) return false;	// atan2 is undefined if both arguments == 0. This implies non reachable desired position.

	_target_angle_rad_2 = -atan2(S_q2, C_q2);

	aux_val1 = (_links_lengths(1) * S_q2);
	aux_val2 = (_links_lengths(0) + _links_lengths(1) * C_q2);
	if (0 == aux_val1 & 0 == aux_val2) return false;	// atan2 is undefined if both arguments == 0. This implies non reachable desired position.

	_target_angle_rad_1 = - (atan2(-_desired_position(1), _desired_position(0)) - atan2( aux_val1, aux_val2));

	return true;
}
