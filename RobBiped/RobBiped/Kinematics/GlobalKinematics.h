/*
 * GlobalKinematics.h
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

#ifndef _GLOBAL_KINEMATICS_h
#define _GLOBAL_KINEMATICS_h

#include "../Main/Configs.h"
#include "../Main/I_PeriodicTask.h"
#include "../Utils/LinearAlgebra/ArduinoEigenDense.h"

using Eigen::Vector3d;

class GlobalKinematics : public I_PeriodicTask {

	public:
		// Walking phases:
		// DSP_left -> DSP and leaning weight towards the left foot (moving to the left)
		// DSP_right -> DSP and leaning weight towards the right foot (moving to the right)
		// SSP_left -> SSP with weight on the left foot
		// SSP_right -> SSP with weight on the right foot
		enum class PosePhases {DSP_left = 0, DSP_right = 1, SSP_left = 2, SSP_right = 3 };

	private:

		// Definitions (configuration)
		Configuration::Configs::Kinematics *kinematics_config_;

		// Phase
		PosePhases phase_;

		// Defined desired hip height
		double desired_hip_height_;

		// Defined desired step width
		double desired_step_width_;

		// Computes the necessary roll angle and leg lengths at home position, for the desired hip height and step width
		void compute_lateral_DSP_home_kinematics();

		// Home-position variables
		double home_roll_angle_;
		double home_leg_length_;

		// Lateral coordinates in ground (Y-axis)
		double right_foot_center_;
		double left_foot_center_;

		// Computed lateral setpoints
		double left_foot_roll_setpoint_, right_foot_roll_setpoint_;
		double left_leg_length_setpoint_, right_leg_length_setpoint_;

		// ...
		Vector3d gyroacc_position_;
		Vector3d CoM_position_;
		Vector3d measured_ZMP_position_;

	public:

		void assoc_config(Configuration::Configs::Kinematics &_config);

		void init(double _centerof_right_foot, PosePhases _phase, double _desired_hip_height, double _desired_step_width);

		void update();

		PosePhases get_walking_phase();

		// Sets desired hip height
		void set_desired_hip_height(double _desired_hip_height);
		// Sets desired step_width
		void set_desired_step_width(double _desired_step_width);
		// Returns the necessary roll joints' angle, for desired step width and hip height, at home position
		double get_home_roll_angle();
		// Returns the necessary legs' length, for desired step width and hip height, at home position
		double get_home_leg_lengths();

		// Returns the defined step width
		double get_step_width();

		// Sets desired hip center position, and computes the necessary roll angle setpoint, and necessary leg lengths, if successful
		bool compute_lateral_DSP_kinematics(const double &_desired_hip_center_position);
		// Returns the last computed roll angles
		void get_computed_angles(double &_left_foot_roll_setpoint, double &_right_foot_roll_setpoint);
		// Returns the last computed leg lengths
		void get_computed_leg_lengths(double &_left_leg_length_setpoint, double &_right_leg_length_setpoint);

	};

#endif
