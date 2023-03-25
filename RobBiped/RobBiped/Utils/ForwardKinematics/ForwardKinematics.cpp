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

void ForwardKinematics::fill_rotation_matrix_from_DH(const Vector4d& DH_row, Matrix4d& TM)
{
	TM(0,0) = cos(DH_row(0));
	TM(0,1) = -sin(DH_row(0)) * cos(DH_row(3));
	TM(0,2) = sin(DH_row(0)) * sin(DH_row(3));

	TM(1,0) = sin(DH_row(0));
	TM(1,1) = cos(DH_row(0)) * cos(DH_row(3));
	TM(1,2) = -cos(DH_row(0)) * sin(DH_row(3));

	TM(2,0) = 0;
	TM(2,1) = sin(DH_row(3));
	TM(2,2) = cos(DH_row(3));
}

void ForwardKinematics::fill_position_from_DH(const Vector4d& DH_row, Matrix4d& TM)
{
	TM(0,3) = DH_row(2) * cos(DH_row(0));	// x
	TM(1,3) = DH_row(2) * sin(DH_row(0));	// y
	TM(2,3) = DH_row(1);					// z
}

void ForwardKinematics::fill_the_rest_of_TM(Matrix4d& TM)
{
	TM(3,0) = 0;
	TM(3,1) = 0;
	TM(3,2) = 0;
	TM(3,3) = 1;
}

void ForwardKinematics::get_TM_from_DH_row(const Vector4d& DH_row, Matrix4d& TM)
{
	fill_rotation_matrix_from_DH(DH_row, TM);
	fill_position_from_DH(DH_row, TM);
	fill_the_rest_of_TM(TM);
}

void ForwardKinematics::get_overall_TM_from_DH_table(const std::vector<Vector4d> &DH_table, Matrix4d& TMf)
{
	TMf = Matrix4d::Identity();
	Matrix4d TM = Matrix4d::Identity();

	for (auto row : DH_table)
	{
		get_TM_from_DH_row(row, TM);
		TMf *= TM;
	}
}
