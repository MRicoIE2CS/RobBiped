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

namespace InverseKinematics {

/*
*  @brief Inverse kinematics by the geometric method,
*  for a kinematic chain formed of two links moving only within the sagittal plane.
*
*  Note: This calculations depend on the chosen coordinate frame of the base.
*/
namespace Geometric {

	/*
	*  @fn void sagittal_two_links_inverse_kinematics(
	*				const Vector3d &_desired_position,
	*				const Vector2d &_links_lengths,
	*				double &_target_angle_1,
	*				double &_target_angle_2,
	*				bool _invert_knee_side = false);
	*  @brief Obtains the necessary joint angles, for the final effector to reach a desired position,
	*  in a two-links kinematic chain, moving only within the sagittal plane (two-dimensional movement).
	*  The knee side is calculated to be in the front (according to humanoid movement). To invert this
	*  calculation, a `true` bool value can be passed to the optional _invert_knee_side parameter.
	*
	*  Coordinate frame of the base:
	*  - The forward direction corresponds to the y axis.
	*  - The lateral direction corresponds to the z axis (not being used in this calculation).
	*  - When in home position, the leg is extended through the x axis.
	*
	*  Output angles' sign is positive corresponding to a positive z-axis rotation (in accordance with Denavit-Hartenberg direct
	*  kinematics convention).
	*  Coordinate frame of each link is positioned as:
	*  - Center is positioned on the joints center.
	*  - X-axis is pointing to the extension of the link.
	*  - Y-axis is pointing to the frontal direction (over the x-y plane of the base's coordinate frame).
	*  - Z-axis is pointing to the lateral direction (normal to the x-y plane of the base's coordinate frame).
	*
	*  @param[in] _desired_position Desired position (x, y, z) vector(3) of the final effector relative to the base.
	*  @param[in] _links_lengths Links lengths, in mm.
	*  @param[in] _invert_knee_side (optional: Default false) If true, the calculated angles correspond to the inverted knee side position.
	*  @param[out] _target_angle_rad_1 Target angle necessary in the first joint to reach the desired position, in radians.
	*  @param[out] _target_angle_rad_2 Target angle necessary in the second joint to reach the desired position, in radians.
	*  @return False if desired position is non reachable. True if successful calculation.
	*/
	bool sagittal_two_links_inverse_kinematics(
			const Vector3d &_desired_position,
			const Vector2d &_links_lengths,
			double &_target_angle_rad_1,
			double &_target_angle_rad_2,
			const bool _invert_knee_side = false);

}	// End namespace Geometric	
}	// End namespace InverseKinematics

#endif
