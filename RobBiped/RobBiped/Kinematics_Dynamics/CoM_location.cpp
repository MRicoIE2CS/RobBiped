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

void CoMLocation::set_CoM_height(const double &_CoM_height)
{
	CoM_location_(2) = _CoM_height;
}

Vector3d CoMLocation::compute_position_from_LIPM(const Vector3d &_CoM_acceleration_measurements_ms2_xyz, const Vector2d &_CoM_inclination_xy, const Vector2d &_ZMP_position_xy)
{
	CoM_inclination_xy_ = _CoM_inclination_xy;

	Vector3d corrected_CoM_acceleration_xyz;
	CoM_acceleration_ = corrected_CoM_acceleration_xyz;

	// From LIP Model:
	double h = CoM_location_(2);
	CoM_location_(0) = h / gravity_constant_ * (_CoM_acceleration_measurements_ms2_xyz(0) *1000.0) + _ZMP_position_xy(0);
	CoM_location_(1) = h / gravity_constant_ * (_CoM_acceleration_measurements_ms2_xyz(1) *1000.0) + _ZMP_position_xy(1);
	
// 	Serial.println("__DEBUG___");
// 	Serial.println("_acc_measure_xyz: \t" + (String)_CoM_acceleration_measurements_xyz(0) + "\t" + (String)_CoM_acceleration_measurements_xyz(1) + "\t" + (String)_CoM_acceleration_measurements_xyz(2));
// 	Serial.println("_corrected_acc_xyz: \t" + (String)corrected_CoM_acceleration_xyz(0) + "\t" + (String)corrected_CoM_acceleration_xyz(1) + "\t" + (String)corrected_CoM_acceleration_xyz(2));
// 	Serial.println("CoM_incl_xy: \t" + (String)_CoM_inclination_xy(0) + "\t" + (String)_CoM_inclination_xy(1));
// 	Serial.println("ZMP_xy: \t" + (String)_ZMP_position_xy(0) + "\t" + (String)_ZMP_position_xy(1));
// 	
// 	Serial.println("CoM_xy: \t" + (String)CoM_location_(0) + "\t" + (String)CoM_location_(1));
	
	return CoM_location_;
}
