/*
 * LegLength_IK.cpp
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

#include "LegLength_IK.h"

#include "../../LinearAlgebra/ArduinoEigenDense.h"

#include "../ForwardKinematics.h"
#include "../GeometricInverseKinematics.h"

using Eigen::Matrix2d;
using Eigen::Vector3d;

bool InverseKinematics::get_leg_joints_angles_from_desired_length_and_orientation(
												const double& _desired_leg_length,
												const double& _forward_angle_rad,
												const double& _first_link_length,
												const double& _second_link_length,
												double& _first_joint_target_angle,
												double& _second_joint_target_angle,
												double& _final_effector_target_angle)
{
	double lateral_angle_rad = 0.0; // Not used
	Vector3d desired_position;
	ForwardKinematics::Geometric::get_position_from_length_and_angles(_desired_leg_length, _forward_angle_rad, lateral_angle_rad, desired_position);

	Vector2d links_lengths;
	links_lengths << _first_link_length, _second_link_length;
	bool ret_val = InverseKinematics::Geometric::sagittal_two_links_inverse_kinematics(desired_position, links_lengths, _first_joint_target_angle, _second_joint_target_angle);

	if (ret_val)
	{
		// Denavit-Hartenberg table
		std::vector<Vector4d> DH_table;
		Vector4d DH_row_1;
		DH_row_1 << _first_joint_target_angle, 0, _first_link_length, 0;
		Vector4d DH_row_2;
		DH_row_2 << _second_joint_target_angle, 0, _second_link_length, 0;
		DH_table.push_back(DH_row_1);
		DH_table.push_back(DH_row_2);

		// Obtain hip joint angle compensation to maintain torso orientation
		Matrix4d TM;
		ForwardKinematics::DenavitHartenberg::get_overall_TM_from_DH_table(DH_table, TM);
		Matrix2d rotation_matrix = TM.block<2,2>(0,0);
		_final_effector_target_angle = atan2(TM(0,1), TM(0,0));
	}

	return ret_val;
}