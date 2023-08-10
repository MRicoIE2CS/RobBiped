/*
 * CoM_location.h
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


#ifndef _COM_LOCATION_h
#define _COM_LOCATION_h

#include "Arduino.h"

#include "../Main/Configs.h"
#include "../Utils/LinearAlgebra/ArduinoEigenDense.h"

using Eigen::Vector2d;
using Eigen::Vector3d;

class CoMLocation {

	private:

		Vector3d CoM_location_;
		Vector3d CoM_velocity_;
		Vector3d CoM_acceleration_;
		Vector2d CoM_inclination_xy_;

		// Gravity constant, in mm/s^2
		const double gravity_constant_ = 9807;

	public:

		// Set CM height.
		// CoM is considered restricted to an horizontal plane, in LIPM
		void set_CoM_height(const double &_CM_height);

		// Compute position from Linear Inverted Pendulum Model
		Vector3d compute_position_from_LIPM(const Vector3d &_CoM_acceleration_measurements_ms2_xyz, const Vector2d &_CoM_inclination_xy, const Vector2d &_ZMP_position_xy);
};

#endif
