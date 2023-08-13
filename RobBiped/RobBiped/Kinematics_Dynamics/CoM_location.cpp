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

void CoMLocation::init_location(Vector3d &_initial_location)
{
	last_CoM_location_ = _initial_location;
	last_millis_ = millis();
}

void CoMLocation::set_CoM_height(const double &_CoM_height)
{
	last_CoM_location_(2) = _CoM_height;
}

void CoMLocation::set_filter_complement_k(const double &_kf)
{
	Kf = _kf;
}

Vector2d CoMLocation::compute_location_from_LIPM(Vector3d &_acc_measure_mms2_xyz, Vector2d &_ZMP_position_xy)
{
	Vector2d com_location;
	double h = last_CoM_location_(2);
	com_location(0) = h / gravity_constant_ * _acc_measure_mms2_xyz(0) + _ZMP_position_xy(0);
	com_location(1) = h / gravity_constant_ * _acc_measure_mms2_xyz(1) + _ZMP_position_xy(1);
	
	return com_location;
}

Vector2d CoMLocation::compute_location_from_integration(Vector3d &_acc_mean_mms2_xyz, double &_time_incr)
{
	Vector2d final_velocity;
	final_velocity(0) = last_CoM_velocity_(0) + _acc_mean_mms2_xyz(0) * _time_incr;
	final_velocity(1) = last_CoM_velocity_(1) + _acc_mean_mms2_xyz(1) * _time_incr;

	Vector2d mean_velocity;
	mean_velocity(0) = last_CoM_velocity_(0) + (final_velocity(0) - last_CoM_velocity_(0)) / 2.0;
	mean_velocity(1) = last_CoM_velocity_(1) + (final_velocity(1) - last_CoM_velocity_(1)) / 2.0;

	// Position incrementation
	Vector2d com_location;
	com_location(0) = last_CoM_location_(0) + mean_velocity(0) * _time_incr;
	com_location(1) = last_CoM_location_(1) + mean_velocity(1) * _time_incr;

	return com_location;
}

Vector3d CoMLocation::compute_location(Vector3d &_CoM_acceleration_measurements_ms2_xyz, Vector2d &_ZMP_position_xy)
{
	// Adequate inputs
	uint32_t current_millis = millis();
	double time_incr = static_cast<double>(current_millis - last_millis_)/1000.0;	// (s)
	Vector3d new_acc_measure_mms2_xyz;
	new_acc_measure_mms2_xyz(0) = _CoM_acceleration_measurements_ms2_xyz(0)*1000.0;
	new_acc_measure_mms2_xyz(1) = _CoM_acceleration_measurements_ms2_xyz(1)*1000.0;
	new_acc_measure_mms2_xyz(2) = _CoM_acceleration_measurements_ms2_xyz(2)*1000.0;
	Vector3d mean_acc_mms2;
	mean_acc_mms2(0) = last_CoM_acceleration_(0) + (new_acc_measure_mms2_xyz(0) - last_CoM_acceleration_(0)) / 2.0;
	mean_acc_mms2(1) = last_CoM_acceleration_(1) + (new_acc_measure_mms2_xyz(1) - last_CoM_acceleration_(1)) / 2.0;
	mean_acc_mms2(2) = last_CoM_acceleration_(2) + (new_acc_measure_mms2_xyz(2) - last_CoM_acceleration_(2)) / 2.0;

	// Computation from each method
	Vector2d intgr_com_location = compute_location_from_integration(mean_acc_mms2, time_incr);
	Vector2d lipm_com_location = compute_location_from_LIPM(new_acc_measure_mms2_xyz, _ZMP_position_xy);

	// Fusion
	Vector3d fused_com_location;
	fused_com_location(0) = Kf * intgr_com_location(0,0) + (1.0-Kf) * lipm_com_location(0);
	fused_com_location(1) = Kf * intgr_com_location(1,0) + (1.0-Kf) * lipm_com_location(1);

	// Update memory values
	last_CoM_acceleration_(0) = new_acc_measure_mms2_xyz(0);
	last_CoM_acceleration_(1) = new_acc_measure_mms2_xyz(1);
	last_CoM_velocity_(0) = (last_CoM_location_(0) - fused_com_location(0)) / time_incr;
	last_CoM_velocity_(1) = (last_CoM_location_(1) - fused_com_location(1)) / time_incr;
	last_CoM_location_(0) = fused_com_location(0);
	last_CoM_location_(1) = fused_com_location(1);
	last_millis_ = current_millis;
	
// 	Serial.println("__DEBUG___");
 	//Serial.println("a, itgr, lipm, fusd, zmp: \t" + (String)new_acc_measure_mms2_xyz(1) + "\t" + (String)intgr_com_location(1) + "\t" + (String)lipm_com_location(1) + "\t" + (String)fused_com_location(1) + "\t" + (String)_ZMP_position_xy(1));
	//Serial.println("a, itgr, lipm, fusd, zmp: \t" + (String)new_acc_measure_mms2_xyz(0) + "\t" + (String)intgr_com_location(0) + "\t" + (String)lipm_com_location(0) + "\t" + (String)fused_com_location(0) + "\t" + (String)_ZMP_position_xy(0));

// 	//Serial.println("_acc_measure_xyz: \t" + (String)_CoM_acceleration_measurements_xyz(0) + "\t" + (String)_CoM_acceleration_measurements_xyz(1) + "\t" + (String)_CoM_acceleration_measurements_xyz(2));
// 	//Serial.println("_corrected_acc_xyz: \t" + (String)corrected_CoM_acceleration_xyz(0) + "\t" + (String)corrected_CoM_acceleration_xyz(1) + "\t" + (String)corrected_CoM_acceleration_xyz(2));
// 	Serial.println("Kf: \t" + (String)Kf);
// 	Serial.println("acc_measure_mms2_xyz: \t" + (String)acc_measure_mms2_xyz(1));
// 	Serial.println("intgr_com_location: \t" + (String)intgr_com_location(1));
// 	Serial.println("lipm_com_location: \t" + (String)lipm_com_location(1));
// 	Serial.println("fused_com_location: \t" + (String)fused_com_location(1));
// 	Serial.println("_ZMP_position_xy: \t" + (String)_ZMP_position_xy(1));
// 
// 	Serial.print("time_incr: \t");
// 	Serial.println(time_incr, 5);
	
	return last_CoM_location_;
}

Vector3d CoMLocation::get_location()
{
	return last_CoM_location_;
}
