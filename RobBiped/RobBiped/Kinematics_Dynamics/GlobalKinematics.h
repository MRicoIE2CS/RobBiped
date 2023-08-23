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

#include "Arduino.h"

#include "CoM_location.h"
//#include "WalkingPhasesManager.h"
#include "../Main/Configs.h"
#include "../Main/I_PeriodicTask.h"
#include "../Utils/Kinematics/InverseKinematicsBlocks/LegLength_IK.h"
#include "../Utils/LinearAlgebra/ArduinoEigenDense.h"
#include "../Utils/Filters/ExponentialFilter.h"

using Eigen::Vector2d;
using Eigen::Vector3d;

// Forward declarations
class ForceSensorsManager;
class GyroscopeAccelerometerManager;

class GlobalKinematics : public I_PeriodicTask {
	public:
		// Walking phases:
		// DSP_left -> DSP and leaning weight towards the left foot (moving to the left)
		// DSP_right -> DSP and leaning weight towards the right foot (moving to the right)
		// SSP_left -> SSP with weight on the left foot
		// SSP_right -> SSP with weight on the right foot
		enum class WalkingPhase {DSP_left = 0, DSP_right = 1, SSP_left = 2, SSP_right = 3 };

	private:

		// Configuration
		Configuration::Configs::Kinematics *config_;

		// Sensors access
		ForceSensorsManager *force_sensors_manager_;
		GyroscopeAccelerometerManager *gyroscope_accelerometer_manager_;

		// Phase
		//WalkingPhasesManager phases_manager_;
		WalkingPhase phase_;

		// Defined desired hip height
		double desired_hip_height_;

		// Defined desired step width
		double desired_step_width_;

		// Location of the Center of Mass
		CoMLocation CoM_location_;
		ExpFilter filter_CoM_location_;

		// Returns the estimated accelerations of the CoM, considering the inclination of the torso
		Vector3d correct_acceleration_inclination(const Vector3d &_CoM_acceleration_measurements_xyz, const Vector2d &_CoM_inclinations);

		// Home-position variables
		double home_roll_angle_;
		double home_leg_length_;

		// Coordinates in ground, for each feet
		double right_foot_center_x_, left_foot_center_x_ = 0.0;
		double right_foot_center_y_, left_foot_center_y_ = 0.0;

		// Computes the necessary roll angle and leg lengths at home position, for the desired hip height and step width
		void compute_lateral_DSP_home_kinematics();

		// Computed lateral roll angle setpoints
		double left_foot_roll_setpoint_, right_foot_roll_setpoint_;
		// Computed leg length setpoints
		// These lengths include all length of the leg, without foot roll joint height (from hip roll joint to foot roll joint)
		double left_leg_length_setpoint_, right_leg_length_setpoint_;

		// Check whether global ZMP is over left footprint
		bool is_zmp_over_left_footprint();
		// Check whether global ZMP is over right footprint
		bool is_zmp_over_right_footprint();
		// Flags to know if any foot have been lifted during SSP
		bool has_left_foot_been_lifted = false;
		bool has_right_foot_been_lifted = false;
		// Flag for if there has been a phase change within this execution period
		bool has_there_been_a_phase_change_ = false;

	public:

		void assoc_config(Configuration::Configs::Kinematics &_config);
		void assoc_sensors(ForceSensorsManager &_force_sensors_manager, GyroscopeAccelerometerManager &_gyroscope_accelerometer_manager);

		void init(double _centerof_right_foot, WalkingPhase _phase, double _desired_hip_height, double _desired_step_width);

		// Sets desired hip height
		bool set_desired_hip_height(double _desired_hip_height);
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
		bool compute_lateral_DSP_kinematics(const double _desired_hip_center_position);
		// Returns the last computed roll angles
		void get_computed_angles(double &_left_foot_roll_setpoint, double &_right_foot_roll_setpoint);
		// Returns the last computed leg lengths. Distance from hip roll joint to ankle roll joint
		void get_computed_leg_lengths(double &_left_leg_length_setpoint, double &_right_leg_length_setpoint);
		// Returns the last computed prismatic lengths. Distances from hip pitch joint to ankle pitch joint
		bool get_computed_prismatic_lengths(double &_left_prismatic_length_setpoint, double &_right_prismatic_length_setpoint);
		// Transforms the desired leg length, from roll joints of hip and ankle, in desired prismatic distance, from pitch joints of hip and ankle
		bool get_prismatic_lenght(const double &_desired_leg_length, double &_desired_prismatic_length);

		// Computes forward and inverse kinematics to obtain joint angles from desired prismatic length (leg length,
		// from hip pitch joint to ankle pitch joint) and desired forward inclination angle
		// Prismatic length allowed range: [115-140]
		// TODO: Get limits from configuration
		bool get_joint_angles_for_prismatic_length(const double &_desired_prismatic_length, const double &_desired_forward_inclination_angle,
											double &_down_joint, double &_mid_joint, double &_up_joint);

		// Returns the distance to the left foot from the right foot
		void get_feet_distance(double &_frontal_distance, double &_lateral_distance);
		// Returns the coordinates of the right foot
		void get_right_foot_coordinates(double &_x, double &_y);
		// Returns the coordinates of the left foot
		void get_left_foot_coordinates(double &_x, double &_y);

		// Returns the angle setpoint for hip roll, with compensation for the deadband (dead zone)
		double compensate_hip_roll_angle(double _desired_hip_roll_angle, bool _left_or_right);

		// Returns the CoM location
		void init_CoM_location();
		// Computes the CoM location
		Vector3d compute_CoM_location();
		// Returns the CoM location
		Vector3d get_CoM_location();
		// Returns the CoM velocity
		Vector3d get_CoM_velocity();
		// Returns the CoM acceleration
		Vector3d get_CoM_acceleration();

		// Checks whether to change the walking phase between DSP and SSP
		bool check_walking_phase();
		// Returns the current walking phase
		WalkingPhase get_current_walking_phase();
		// Sets the current walking phase
		void force_current_walking_phase(WalkingPhase _phase);
		// Returns true if there has been a phase change within this execution period
		bool has_there_been_a_phase_change();
	};

#endif
