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
#include "../Utils/Kinematics/InverseKinematicsBlocks/LegLength_IK.h"
#include "../Utils/LinearAlgebra/ArduinoEigenDense.h"

using Eigen::Vector3d;
using Eigen::Vector4d;

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
		Configuration::Configs::Kinematics *config_;

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

		// Coordinates in ground, for each feet
		double right_foot_center_x_, left_foot_center_x_ = 0.0;
		double right_foot_center_y_, left_foot_center_y_ = 0.0;

		// Computed lateral roll angle setpoints
		double left_foot_roll_setpoint_, right_foot_roll_setpoint_;
		// Computed leg length setpoints
		// These lengths include all length of the leg, without foot roll joint height (from hip roll joint to foot roll joint)
		double left_leg_length_setpoint_, right_leg_length_setpoint_;

	public:

		void assoc_config(Configuration::Configs::Kinematics &_config);

		void init(double _centerof_right_foot, PosePhases _phase, double _desired_hip_height, double _desired_step_width);

		PosePhases get_walking_phase();

		// Sets desired hip height
		void set_desired_hip_height(double _desired_hip_height);
		// Sets desired step_width
		void set_desired_step_width(double _desired_step_width);
		// Returns the necessary roll joint angles, for desired step width and hip height, at home position
		double get_home_roll_angle();
		// Returns the necessary legs lengths, for desired step width and hip height, at home position
		// These lengths include all length of the leg, without foot roll joint height (from hip roll joint to foot roll joint)
		double get_home_leg_lengths();

		// Returns the defined step width
		double get_step_width();

		// Sets desired hip center position, and computes the necessary roll angle setpoint, and necessary leg lengths, if successful
		bool compute_lateral_DSP_kinematics(const double &_desired_hip_center_position);
		// Returns the last computed roll angles
		void get_computed_angles(double &_left_foot_roll_setpoint, double &_right_foot_roll_setpoint);
		// Returns the last computed leg lengths
		void get_computed_leg_lengths(double &_left_leg_length_setpoint, double &_right_leg_length_setpoint);

		// Computes forward and inverse kinematics to obtain joint angles from desired prismatic length (leg length,
		// from hip pitch joint to ankle pitch joint) and desired forward inclination angle
		// Prismatic length allowed range: [115-140]
		// TODO: Get limits from configuration
		bool get_joint_angles_for_leg_length(const double &_desired_prismatic_length, const double &_desired_forward_inclination_angle,
											double &_down_joint, double &_mid_joint, double &_up_joint);

		// Returns the distance to the left foot from the right foot
		void get_feet_distance(double &_frontal_distance, double &_lateral_distance);
		// Returns the coordinates of the right foot
		void get_right_foot_coordinates(double &_x, double &_y);
		// Returns the coordinates of the left foot
		void get_left_foot_coordinates(double &_x, double &_y);

		double compensate_hip_roll_angle(double &_desired_hip_roll_angle);
	};

#endif
