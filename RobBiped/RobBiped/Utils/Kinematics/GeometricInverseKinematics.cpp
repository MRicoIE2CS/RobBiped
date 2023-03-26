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

void InverseKinematics::Geometric::get_desired_position_from_length_and_angles(
		const double &_leg_length_mm, const double &_forward_angle_rad, const double &_lateral_angle_rad,
		Vector3d &_desired_position)
{
	_desired_position(0) = _leg_length_mm * cos(_forward_angle_rad) * cos(_lateral_angle_rad);
	_desired_position(1) = _leg_length_mm * sin(_forward_angle_rad) * cos(_lateral_angle_rad);
	_desired_position(2) = _leg_length_mm * cos(_forward_angle_rad) * sin(_lateral_angle_rad);
}

void InverseKinematics::Geometric::sagittal_two_links_inverse_kinematics(
		const Vector3d &_desired_position,
		const Vector2d &_links_lengths,
		double &_target_angle_rad_1,
		double &_target_angle_rad_2)
{
	double C_q2 = (pow(_desired_position(0), 2) + pow(_desired_position(1), 2) - pow(_links_lengths(0), 2) - pow(_links_lengths(1), 2) ) / (2 * _links_lengths(0) * _links_lengths(1));
	double S_q2 = sqrt(1 - pow(C_q2, 2));

	_target_angle_rad_2 = atan2(S_q2, C_q2);

	_target_angle_rad_1 = atan2(_desired_position(1), _desired_position(0)) - atan2( (_links_lengths(1) * S_q2), (_links_lengths(0) + _links_lengths(1) * C_q2));
}
