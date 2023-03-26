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

void ForwardKinematics::fill_rotation_matrix_from_DH(const Vector4d& _DH_row, Matrix4d& _TM)
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

void ForwardKinematics::fill_position_from_DH(const Vector4d& _DH_row, Matrix4d& _TM)
{
	_TM(0,3) = _DH_row(2) * cos(_DH_row(0));	// x
	_TM(1,3) = _DH_row(2) * sin(_DH_row(0));	// y
	_TM(2,3) = _DH_row(1);						// z
}

void ForwardKinematics::fill_the_rest_of_TM(Matrix4d& _TM)
{
	_TM(3,0) = 0;
	_TM(3,1) = 0;
	_TM(3,2) = 0;
	_TM(3,3) = 1;
}

void ForwardKinematics::get_TM_from_DH_row(const Vector4d& _DH_row, Matrix4d& _TM)
{
	fill_rotation_matrix_from_DH(_DH_row, _TM);
	fill_position_from_DH(_DH_row, _TM);
	fill_the_rest_of_TM(_TM);
}

void ForwardKinematics::get_overall_TM_from_DH_table(const std::vector<Vector4d> &_DH_table, Matrix4d& _TMf)
{
	_TMf = Matrix4d::Identity();
	Matrix4d TM = Matrix4d::Identity();

	for (auto row : _DH_table)
	{
		get_TM_from_DH_row(row, TM);
		_TMf *= TM;
	}
}
