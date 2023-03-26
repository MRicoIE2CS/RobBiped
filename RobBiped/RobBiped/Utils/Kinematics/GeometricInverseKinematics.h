/*
 * GeometricInverseKinematics.h
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

#ifndef _GEOMETRICINVERSEKINEMATICS_h
#define _GEOMETRICINVERSEKINEMATICS_h

#include "arduino.h"

#include "../LinearAlgebra/ArduinoEigenDense.h"

using Eigen::Vector3d;
using Eigen::Vector2d;

/*
*  @brief Inverse kinematics by the geometric method,
*  for a kinematic chain formed of two links moving only within the sagittal plane.
*/
namespace InverseKinematics {
namespace Geometric {	

	/*
	*  @fn void get_desired_position_from_length_and_angles(
	*				const long &leg_length_mm, const double &frontal_angle_rad, const double &lateral_angle_rad,
	*				Vector3d &desired_position);
	*  @brief Obtains the desired position (x, y, z) of the final effector, from the desired leg length and
	*  the desired frontal and lateral angles of the final effector relative to the base coordinate frame.
	*
	*  @param[in] leg_length_mm Desired leg length, in mm.
	*  @param[in] frontal_angle_rad Desired frontal angle, in radians.
	*  @param[in] lateral_angle_rad Desired lateral angle, in radians.
	*  @param[out] desired_position Desired position (x, y, z) vector(3) of the final effector relative to the base.
	*/
	void get_desired_position_from_length_and_angles(
			const double &_leg_length_mm, const double &_frontal_angle_rad, const double &_lateral_angle_rad,
			Vector3d &_desired_position);

	/*
	*  @fn void sagittal_two_links_inverse_kinematics(
	*				const Vector3d &_desired_position,
	*				const Vector2d &_links_lengths,
	*				double &_target_angle_1,
	*				double &_target_angle_2);
	*  @brief Obtains the desired position (x, y, z) of the final effector, from the desired leg length and
	*  the desired frontal and lateral angles of the final effector relative to the base coordinate frame,
	*  for a two links kinematic chain, moving only within the sagittal plane.
	*
	*  @param[in] _desired_position Desired position (x, y, z) vector(3) of the final effector relative to the base.
	*  @param[in] _links_lengths Links lengths, in mm.
	*  @param[out] _target_angle_rad_1 Target angle necessary in the first joint to reach the desired position, in radians.
	*  @param[out] _target_angle_rad_2 Target angle necessary in the second joint to reach the desired position, in radians.
	*/
	void sagittal_two_links_inverse_kinematics(
			const Vector3d &_desired_position,
			const Vector2d &_links_lengths,
			double &_target_angle_rad_1,
			double &_target_angle_rad_2);

}	// End namespace Geometric	
}	// End namespace InverseKinematics

#endif
