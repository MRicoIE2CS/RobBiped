/*
 * LegLength_IK.h
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

#ifndef _LEGLENGTH_IK_h
#define _LEGLENGTH_IK_h

#include "arduino.h"

namespace InverseKinematics {

	/*
	*  @fn bool get_supporting_leg_joints_angles_from_desired_length_and_orientation(
	*									const double& _desired_prismatic_length,
	*									const double& _forward_angle,
	*									const double& _first_link_length,
	*									const double& _second_link_length,
	*									double& _first_joint_target_angle,
	*									double& _second_joint_target_angle,
	*									double& _final_effector_target_angle)
	*  @brief Obtains the necessary joint angles, for a leg composed of a three-links kinematic chain,
	*  moving only within the sagittal plane (two-dimensional movement).
	*  The knee side is calculated to be in the front (according to humanoid movement).
	*  Transforms a desired leg length and forward orientation into necessary joint angles for
	*  the pitch joints of the ankle, knee and hip.
	*
	*  Output angles' sign is positive corresponding to a positive z-axis rotation (in accordance with
	*  Denavit-Hartenberg direct kinematics convention), considering that the chain starts from the ground
	*  (supporting leg), and the lengths and joints numbers start counting from there.
	*  Coordinate frame of each link is positioned as:
	*  - Center is positioned on the joints center.
	*  - X-axis is pointing to the extension of the link.
	*  - Y-axis is pointing to the frontal direction (over the x-z plane of the base's coordinate frame).
	*  - Z-axis is pointing to the lateral direction (left, normal to the x-z plane of the base's coordinate frame).
	*
	*  @param[in] _desired_prismatic_length Leg length, from the ankle pitch joint to the hip pitch joint, in mm.
	*  @param[in] _forward_angle Angle of the vector that goes from the ankle pitch joint to the hip pitch joint,
	*  relative to the vertical axis, in rads.
	*  @param[in] _first_link_length Length of the first link, from the ankle pitch joint to the knee pitch joint.
	*  @param[in] _second_link_length Length of the second link, from the knee pitch joint to the hip pitch joint.
	*  @param[out] _first_joint_target_angle Ankle pitch joint angle, in rads.
	*  @param[out] _second_joint_target_angle Knee pitch joint angle, in rads.
	*  @param[out] _final_effector_target_angle Hip pitch joint angle, in rads.
	*  @return False if desired position is non reachable. True if successful calculation.
	*/
	bool get_supporting_leg_joints_angles_from_desired_length_and_orientation(
													const double& _desired_prismatic_length,
													const double& _forward_angle_rad,
													const double& _first_link_length,
													const double& _second_link_length,
													double& _first_joint_target_angle,
													double& _second_joint_target_angle,
													double& _final_effector_target_angle);

}	// End namespace InverseKinematics

#endif