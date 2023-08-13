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

		// Estimated variables (mm | mm/s | mm/s^2)
		Vector3d last_CoM_location_;
		Vector3d last_CoM_velocity_;
		Vector3d last_CoM_acceleration_;

		// Constant of the complement filter for the estimation of the CM location [0.0-1.0]
		double Kf = 1;

		// Gravity constant (mm/s^2)
		const double gravity_constant_ = 9807;

		// Milliseconds when the location was last executed
		uint32_t last_millis_ = millis();

		// Compute position from Linear Inverted Pendulum Model
		Vector2d compute_location_from_LIPM(Vector3d &_acc_measure_mms2_xyz, Vector2d &_ZMP_position_xy);

		// Compute position integrating from accelerometer measurement
		Vector2d compute_location_from_integration(Vector3d &_acc_mean_mms2_xyz, double &_time_incr);

	public:

		// Initiate position
		void init_location(Vector3d &_initial_location);

		// Set CM height.
		// CoM is considered restricted to an horizontal plane, in LIPM
		void set_CoM_height(const double &_CM_height);

		// Set the constant of the complement filter for the estimation of the CM location [0.0-1.0]
		// A value of 1.0, gives all the weight to the calculation from the integration of the acceleration measurements
		// A value of 0.0, gives all the weight to the calculation from the linear inverted pendulum model and the acceleration and ZMP measurements
		void set_filter_complement_k(const double &_kf);

		// Compute position
		Vector3d compute_location(Vector3d &_CoM_acceleration_measurements_ms2_xyz, Vector2d &_ZMP_position_xy);

		// Compute position
		Vector3d get_location();
};

#endif
