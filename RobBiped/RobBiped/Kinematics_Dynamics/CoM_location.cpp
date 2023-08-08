/*
 * CoM_location.cpp
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

#include "CoM_location.h"

void CoMLocation::Set_CoM_height(const double &_CoM_height)
{
	CoM_location_(3) = _CoM_height;
}

Vector3d CoMLocation::correct_acceleration_inclination(const Vector3d &_CoM_acceleration_measurements_xyz, const Vector2d &_CoM_inclination_xy)
{
	double xz_mod = sqrt( pow(_CoM_acceleration_measurements_xyz(1),2) + pow(_CoM_acceleration_measurements_xyz(3),2) );
	double gamma_x = atan2(_CoM_acceleration_measurements_xyz(1), -_CoM_acceleration_measurements_xyz(3));
	double yz_mod = sqrt( pow(_CoM_acceleration_measurements_xyz(2),2) + pow(_CoM_acceleration_measurements_xyz(3),2) );
	double gamma_y = atan2(_CoM_acceleration_measurements_xyz(2), -_CoM_acceleration_measurements_xyz(3));
	Vector3d corrected_CoM_acceleration_xyz;
	corrected_CoM_acceleration_xyz(1) = xz_mod * sin(gamma_x - _CoM_inclination_xy(1));
	corrected_CoM_acceleration_xyz(2) = yz_mod * sin(gamma_y - _CoM_inclination_xy(2));
	corrected_CoM_acceleration_xyz(3) = xz_mod * cos(gamma_x - _CoM_inclination_xy(1));
}

Vector3d CoMLocation::compute_position_from_LIPM(const Vector3d &_CoM_acceleration_measurements_xyz, const Vector2d &_CoM_inclination_xy, const Vector2d &_ZMP_position_xy)
{
	Vector3d corrected_CoM_acceleration_xyz;
	corrected_CoM_acceleration_xyz = correct_acceleration_inclination(_CoM_acceleration_measurements_xyz, _CoM_inclination_xy);
	CoM_acceleration_ = corrected_CoM_acceleration_xyz;
	CoM_inclination_xy_ = _CoM_inclination_xy;

	// From LIP Model:
	double h = CoM_location_(3);
	CoM_location_(1) = h / gravity_constant_ * _CoM_acceleration_measurements_xyz(1) + _ZMP_position_xy(1);
	CoM_location_(2) = h / gravity_constant_ * _CoM_acceleration_measurements_xyz(2) + _ZMP_position_xy(2);
	
	return CoM_location_;
}
