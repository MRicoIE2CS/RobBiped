/*
 * ForwardKinematics.h
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

#ifndef _FORWARDKINEMATICS_h
#define _FORWARDKINEMATICS_h

#include "arduino.h"

#include <vector>

#include "../LinearAlgebra/ArduinoEigenDense.h"

using Eigen::Matrix4d;
using Eigen::Vector4d;
using Eigen::Vector3d;

/*
*  @brief Forward kinematics by the Denavit-Hartenberg method.
*
*  An homogeneous transformation matrix defines a translation and a rotation.
*  TM = [ R11 R12 R13 X;
*		  R21 R22 R23 Y;
*		  R31 R32 R33 Z;
*		  0   0   0   1 ]
*
*  Each transformation matrix is defined from four parameters.
*  The overall forward kinematics are defined in a DH table, on which each row defines
*  the translation and rotation of each link of the kinematic chain.
*
*  Note: This calculations depend on the chosen coordinate frame of the base.
*/
namespace ForwardKinematics {

	/*
	*  @fn void get_rotation_matrix_from_DH(const Vector4d& DH_row, Matrix3d& R)
	*  @brief Fills the rotation matrix of the homogeneous transformation matrix,
	*  from one row of the Denavit-Hartenberg table (one Denavit-Hartenberg transformation).
	*
	*  @param[in] _DH_row Denavit-Hartenberg row.
	*  @param[out] _TM Homogeneous transformation matrix.
	*/
	void fill_rotation_matrix_from_DH(const Vector4d& _DH_row, Matrix4d& _TM);

	/*
	*  @fn void get_position_from_DH(const Vector4d& DH_row,  Vector3d& pos)
	*  @brief Fills the position coordinates (x, y, z) of the homogeneous transformation matrix,
	*  from one row of the Denavit-Hartenberg table (one Denavit-Hartenberg transformation).
	*
	*  @param[in] _DH_row Denavit-Hartenberg row.
	*  @param[out] _TM Homogeneous transformation matrix.
	*/
	void fill_position_from_DH(const Vector4d& _DH_row, Matrix4d& _TM);

	/*
	*  @fn void fill_the_rest_of_TF(const Vector4d& DH_row,  Vector3d& pos)
	*  @brief Fills the last row of the homogeneous transformation matrix [perspective (0,0,0) and scale(1)].
	*
	*  @param[out] _TM Homogeneous transformation matrix.
	*/
	void fill_the_rest_of_TM(Matrix4d& _TM);

	/*
	*  @fn void get_TM_from_DH_row(const Vector4d& DH_row,  Matrix4d& TF)
	*  @brief Obtains the homogeneous transformation matrix
	*  from one row of the Denavit-Hartenberg table (one Denavit-Hartenberg transformation).
	*
	*  @param[in] _DH_row One row of the Denavit-Hartenberg table.
	*  @param[out] _TM Homogeneous transformation matrix.
	*/
	void get_TM_from_DH_row(const Vector4d& _DH_row, Matrix4d& _TM);

	/*
	*  @fn void get_overall_TM_from_DH_table(const std::vector<Vector4d> &DH_table,  Matrix4d& TF)
	*  @brief Obtains the homogeneous transformation matrix from the base to the final effector
	*  from the Denavit-Hartenberg table.
	*
	*  @param[in] _DH_table Denavit-Hartenberg table.
	*  @param[out] _TMf Homogeneous transformation matrix.
	*/
	void get_overall_TM_from_DH_table(const std::vector<Vector4d> &_DH_table, Matrix4d& _TMf);

	/*
	*  @fn void get_length_and_angles_from_position(
	*				const Vector3d &_position,
	*				double &_leg_length_mm, double &_frontal_angle_rad, double &_lateral_angle_rad);
	*  @brief Obtains the leg length and the frontal angle (against the vertical axis) of the final effector
	*  from the position (x, y, z) of the final effector, relative to the base coordinate frame.
	*
	*  Coordinate frame of the base:
	*  The forward direction corresponds to the y axis.
	*  The lateral direction corresponds to the z axis.
	*  When in home position, the leg is extended through the x axis.
	*
	*  @param[in] _position Position (x, y, z) vector(3) of the final effector relative to the base.
	*  @param[out] _leg_length_mm Leg length, in mm.
	*  @param[out] _forward_angle_rad Frontal angle, in radians.
	*  @param[out] _lateral_angle_rad Lateral angle, in radians.
	*/
	void get_length_and_angles_from_position(
			const Vector3d &_position,
			double &_leg_length_mm, double &_forward_angle_rad, double &_lateral_angle_rad);

}	// End namespace ForwardKinematics

#endif
