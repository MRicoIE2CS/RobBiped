/*
 * ForwardKinematics.cpp
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

#include "ForwardKinematics.h"

#include <math.h>

void ForwardKinematics::DenavitHartenberg::fill_rotation_matrix_from_DH(const Vector4d& _DH_row, Matrix4d& _TM)
{
	_TM(0,0) = cos(_DH_row(0));
	_TM(0,1) = -sin(_DH_row(0)) * cos(_DH_row(3));
	_TM(0,2) = sin(_DH_row(0)) * sin(_DH_row(3));

	_TM(1,0) = sin(_DH_row(0));
	_TM(1,1) = cos(_DH_row(0)) * cos(_DH_row(3));
	_TM(1,2) = -cos(_DH_row(0)) * sin(_DH_row(3));

	_TM(2,0) = 0;
	_TM(2,1) = sin(_DH_row(3));
	_TM(2,2) = cos(_DH_row(3));
}

void ForwardKinematics::DenavitHartenberg::fill_position_from_DH(const Vector4d& _DH_row, Matrix4d& _TM)
{
	_TM(0,3) = _DH_row(2) * cos(_DH_row(0));
	_TM(1,3) = _DH_row(2) * sin(_DH_row(0));
	_TM(2,3) = _DH_row(1);
}

void ForwardKinematics::DenavitHartenberg::fill_the_rest_of_TM(Matrix4d& _TM)
{
	_TM(3,0) = 0;
	_TM(3,1) = 0;
	_TM(3,2) = 0;
	_TM(3,3) = 1;
}

void ForwardKinematics::DenavitHartenberg::get_TM_from_DH_row(const Vector4d& _DH_row, Matrix4d& _TM)
{
	fill_rotation_matrix_from_DH(_DH_row, _TM);
	fill_position_from_DH(_DH_row, _TM);
	fill_the_rest_of_TM(_TM);
}

void ForwardKinematics::DenavitHartenberg::get_overall_TM_from_DH_table(const std::vector<Vector4d> &_DH_table, Matrix4d& _TMf)
{
	_TMf = Matrix4d::Identity();
	Matrix4d TM = Matrix4d::Identity();

	for (auto row : _DH_table)
	{
		get_TM_from_DH_row(row, TM);
		_TMf *= TM;
	}
}

void ForwardKinematics::Geometric::get_position_from_length_and_angles(
const double &_leg_length_mm, const double &_forward_angle_rad, const double &_lateral_angle_rad,
Vector3d &_desired_position)
{
	_desired_position(0) = _leg_length_mm * sin(_forward_angle_rad);
	_desired_position(1) = _leg_length_mm * sin(_lateral_angle_rad);
	_desired_position(2) = _leg_length_mm * cos(_forward_angle_rad) * cos(_lateral_angle_rad);
}

// TODO: Check method to comply with conventions
void ForwardKinematics::Geometric::get_length_and_angles_from_position(
		const Vector3d &_position,
		double &_leg_length_mm, double &_forward_angle_rad, double &_lateral_angle_rad)
{
	// Calculation of the inclination in the forward direction (y direction)
	if (0 != _position(1))
	{
		_leg_length_mm = sqrt(pow(_position(0), 2) + pow(_position(1), 2));		// l_s = sqrt( x^2 + y^2 )
		double cos_forward_angle = _position(1) / _leg_length_mm;				// cos(delta) = y / l_s
		double sin_forward_angle = sqrt(1 - pow(cos_forward_angle, 2));			// sin(delta) = sqrt( 1 - cos(delta)^2 )
		_forward_angle_rad = M_PI /2 - atan2(sin_forward_angle, cos_forward_angle);		// delta = PI/2 - atan(sin / cos)
	}
	else
	{
		_leg_length_mm = _position(0);
		_forward_angle_rad = 0.0;
	}
	
	// Calculation of the inclination in the lateral direction (z direction),
	// and the final leg length
	if (0 != _position(2))	// If z-component is not 0
	{
		_leg_length_mm = sqrt(pow(_leg_length_mm, 2) + pow(_position(2), 2));		// l_f = sqrt( l_s^2 + z^2 )
		double l_lateral = sqrt(pow(_position(0), 2) + pow(_position(2), 2));	// l_l = sqrt( x^2 + z^2 )
		double cos_lateral_angle = _position(2) / l_lateral;					// cos(rho) = z / l_l
		double sin_lateral_angle = sqrt(1 - pow(cos_lateral_angle, 2));			// sin(rho) = sqrt( 1 - cos(rho)^2 )
		_lateral_angle_rad = M_PI/2 - atan2(sin_lateral_angle, cos_lateral_angle);		// delta = PI/2 - atan(sin / cos)
	}
	else _lateral_angle_rad = 0.0;
}
