/*
 * Rotation.h
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

#ifndef _ROTATION_h
#define _ROTATION_h

#include "arduino.h"

#include "../../LinearAlgebra/ArduinoEigenDense.h"

using Eigen::Vector3d;
using Eigen::Matrix3d;

namespace Geometry {

	Vector3d rotate_in_xy(const Vector3d &_input_vector, const double &_rotation_angle_roll, const double &_rotation_angle_pitch)
	{
		Vector3d output_vector = _input_vector;

		if (0.0 != _rotation_angle_roll)
		{
			Matrix3d rotation_matrix_x = Matrix3d::Zero();
			rotation_matrix_x(0,0) = 1;
			rotation_matrix_x(1,1) = cos(_rotation_angle_roll);
			rotation_matrix_x(1,2) = -sin(_rotation_angle_roll);
			rotation_matrix_x(2,1) = sin(_rotation_angle_roll);
			rotation_matrix_x(2,2) = cos(_rotation_angle_roll);

			output_vector = rotation_matrix_x * output_vector;
		}

		if (0.0 != _rotation_angle_roll)
		{
			Matrix3d rotation_matrix_y = Matrix3d::Zero();
			rotation_matrix_y(1,1) = 1;
			rotation_matrix_y(0,0) = cos(_rotation_angle_pitch);
			rotation_matrix_y(0,2) = sin(_rotation_angle_pitch);
			rotation_matrix_y(2,0) = -sin(_rotation_angle_pitch);
			rotation_matrix_y(2,2) = cos(_rotation_angle_pitch);

			output_vector = rotation_matrix_y * output_vector;
		}

		return output_vector;
	}

}	// End namespace Geometry

#endif

